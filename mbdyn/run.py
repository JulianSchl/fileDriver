import logging
#from matplotlib import pyplot as plt
import precice
import time
import subprocess
from .rotation import *		#rotation.py
from .csvreader import *	#csvreader.py
import pathlib
import numpy as np
import decimal
from .mbdynAdapter.prep import MBDynPrep
import socket
import errno
from scipy import interpolate
import colorama
import struct
from .socketTools import *# SocketTools as s

class MbdynAdapter:

	#initialize values
	colorama.init(autoreset=True)
	
	from .initialize import __init__
	
	#run preCICE coupling
	def run(self):
		s = SocketTools()
		interface = precice.Interface(self.participant_name, self.configuration_file_name,
				                      self.solver_process_index, self.solver_process_size)

		for i in self.mesh_name:
			self.mesh_id.append(interface.get_mesh_id(str(i)))
		dimensions = interface.get_dimensions()
		
		for i in range(self.patches):
			self.patch.append(Rotation())
			self.vertices.append(csvImport(self.current_path + '/../' + self.fluid_folder_name + '/patch' + str(i+1) + '.csv'))	#if mesh is in file
			self.patch[i].importGrid(self.vertices[i])
			
		if not (len(self.vertices) == len(self.mesh_id)):
			exit("Cannot match mesh to vertices")
		
		#interpolate from beam to airfoil
		for i in range(self.patches):
			x = self.vertices[i][:,0]
			x = np.roll(x, -1)
			x = np.split(x,2)
			y = self.vertices[i][:,1]
			y = np.roll(y, -1)
			y = np.split(y,2)
			y_min = np.array((np.array(y[0]).min(axis=0),np.array(y[1]).min(axis=0))).max(axis=0)
			y_max = np.array((np.array(y[0]).max(axis=0),np.array(y[1]).max(axis=0))).min(axis=0)
			y_center = y_min + (y_max-y_min)/2
			num_of_points = 11
			
			#x = f(y)
			f = [interpolate.interp1d(y[0],x[0]),interpolate.interp1d(y[1],x[1])]
			ynew = [np.linspace(y_min,y_max,11),np.linspace(y_min,y_max,11)]
			xnew = [f[0](ynew[0]),f[1](ynew[1])]
			
			#x[0] rechts
			#x[1] links
			#von links nach rechts
			airfoil_mesh = np.concatenate(np.array((np.array((xnew[1],ynew[1],np.zeros((xnew[1].size)))) ,np.array((xnew[0],ynew[0],np.zeros((xnew[0].size)))))),axis=1)
			self.vertices[i] = airfoil_mesh.T

			self.vertex_ids.append(interface.set_mesh_vertices(self.mesh_id[i],self.vertices[i]))
			self.read_data_id.append(interface.get_data_id(self.read_data_name[i], self.mesh_id[i]))
			self.write_data_id.append(interface.get_data_id(self.write_data_name[i], self.mesh_id[i]))
		
		self.mbdyn.initialize(case=self.mbdyn_prep.name)
		
		print("trying to connect to socket stream...")
		#MBDyn drive caller socket
		self.tvc_socket = s.driveConnect()
		self.exchange_socket = s.patchConnect(self.patches+1)
		print(u'\u2705' + " " + colorama.Style.BRIGHT + colorama.Back.WHITE + colorama.Fore.GREEN + str(self.patches+2) + " socket(s) connected")
		
		dt = interface.initialize()
		
		self.input.update_time_step(dt)
		
		print("preInit")
		self.mbdyn.nodal_initialize()
		#send tvc socket data
		s_out_bufsize = 8
		i = 0
		tvc =  self.iteration * 0.02
		buf_out = bytearray(struct.pack("d",tvc))
		self.tvc_socket[i].sendall(buf_out, s_out_bufsize)
		print("tvc: " + str(tvc))	
		#send tvc socket data
		i = 0
		tvc =  self.iteration * 0.02
		buf_out = bytearray(struct.pack("d",tvc))
		self.tvc_socket[i].sendall(buf_out, s_out_bufsize)
		print("tvc: " + str(tvc))	
		self.mbdyn.nodal_negotiate()
		#TODO: Is first iteration step here?
		self.mbdyn.nodal_negotiate_recv()
		self.transform = Rotation()
		
		self.current_time_step = decimal.Decimal(str(dt))
		
		while interface.is_coupling_ongoing():
			
			if interface.is_action_required(
				    precice.action_write_iteration_checkpoint()):
				print("DUMMY: Writing iteration checkpoint")
				interface.mark_action_fulfilled(
				    precice.action_write_iteration_checkpoint())

			self.read_data.clear()
			self.force_tensor.clear()
			for i in range(self.patches):
				if interface.is_read_data_available():
					self.read_data.append(interface.read_block_vector_data(self.read_data_id[i], self.vertex_ids[i]))
				elif float(self.current_time_step) == float(decimal.Decimal(str(dt))):
					self.read_data.append(self.vertices[i]*0)
				else:
					exit("no data provided!")
				self.force_tensor.append(np.split(self.read_data[i],2)[0]*0) # prepare force_tensor for entries
				self.force_tensor[i] = np.concatenate((self.force_tensor[i],self.force_tensor[i][:2,:]*0)) # prepare force_tensor for entries, add empty anchor & ground force node
				for k,j in enumerate(np.split(self.read_data[i],2)):
					self.force_tensor[i] += np.concatenate((j,j[:2,:]*0)) # fill values into force_tensor, add empty anchor & ground force node
			
			#TODO: divide forces into timesteps
			mbdyn_more_iterations = 10
			
			self.mbdyn.set_forces(np.vstack(self.force_tensor))
			
			tvc = self.iteration * 0.02
			
			#send tvc socket data
			i = 0
			buf_out = bytearray(struct.pack("d",tvc))
			self.tvc_socket[i].sendall(buf_out, s_out_bufsize)
			print("tvc: " + str(tvc))
			
			if self.mbdyn.solve(False):
				self.module_logger.debug('Something went wrong!')
				#MBDyn diverges
				break
				
			npp = self.mbdyn.mesh.number_of_nodes()/self.patches #nodes per patch
			if npp % 2 != 0 and npp % 1 != 0:
				exit("please check number of nodes / patches!")
			npp = int(npp)
			
			self.displacement_absolute_delta.clear()
			self.displacement_absolute_delta_str.clear()
			self.displacement_relative_delta.clear()
			self.rotation_absolute_delta.clear()
			self.rotation_absolute_delta_str.clear()
			self.mbdyn_mesh.clear()
			self.mbdyn_mesh_str.clear()
			self.nodes_absolute.clear()
			self.message.clear()
			
			for i in range(self.patches):
				
				# absolute = MBDyn base mesh as reference
				# absolute delta = MBDyn base mesh as reference, difference between current pos and base mesh
				#evaluate difference between initial mesh and current mesh
				#TODO: add clear commands above
				self.displacement_absolute_delta.append(self.mbdyn.get_absolute_displacement()[i*npp:(i+1)*npp,:]) 
				self.displacement_absolute_delta_str.append(np.char.mod("%.8f",self.displacement_absolute_delta[i]))
				self.rotation_absolute_delta.append(self.mbdyn.get_rotation()[i*npp:(i+1)*npp,:])
				self.rotation_absolute_delta_str.append(np.char.mod("%.8f",self.rotation_absolute_delta[i]))
				#TODO: change to old version with dynamicMeshDict
				self.mbdyn_mesh.append(self.mbdyn.mesh.nodes[i*npp:(i+1)*npp,:])
				self.mbdyn_mesh_str.append(np.char.mod("%.8f",self.mbdyn_mesh[i]))
				self.nodes_absolute.append(self.mbdyn.get_nodes()[i*npp:(i+1)*npp,:])
			
			##############################
			### evaluate OpenFOAM data ###
			##############################
			#import ipdb; ipdb.set_trace()
			for i in range(self.patches+1):
				#distinction between background and overset mesh
				if i == 0:
					#background mesh
					self.message.append(';'.join(np.concatenate((self.displacement_absolute_delta_str[i][npp-1,:],
																	np.char.mod("%.1f",self.rotation_absolute_delta[i][npp-1,:]*0),
																	np.char.mod("%.1f",self.mbdyn_mesh[i][npp-1,:]*0)))))
				else:
					#overset mesh
					self.message.append(';'.join(np.concatenate((self.displacement_absolute_delta_str[i-1][5,:],
																	self.rotation_absolute_delta_str[i-1][5,:],
																	self.mbdyn_mesh_str[i-1][5,:]))))
				print('Sending')
				print(self.message[i])
				print("iteration:" +str(i))
				self.exchange_socket[i].sendall(str.encode(self.message[i]))
					
				print('Sent')
				data = self.exchange_socket[i].recv(250) # 21 chars per message
				if str(data) == "b''":
					for j in range(self.patches+1):
						self.exchange_socket[j].close()
					exit("Client exit")
				split_string = str(data).split("\\n")
				data = split_string[0][2:]
				if str(data) == "exit":
					for j in range(self.patches+1):
						self.exchange_socket[j].close()
					exit("Client exit")
				print('Received', repr(data))
				#import ipdb; ipdb.set_trace()
			
			#############################
			### evaluate preCICE data ###
			#############################
			#time.sleep(2)
			for i in range(self.patches):				
				#transform rotor rotation of most deformed element
				
				#new anchor node positions - ground displacement_absolute_delta
				self.transform.importGrid(self.nodes_absolute[i][:npp-2,:]-self.displacement_absolute_delta[i][:npp-2,:])
				self.transform.rotate(angle=-self.rotation_absolute_delta[i][npp-2,2],rot_point = np.array((0,0,0)))
				
				#TODO: why npp-2 in rotation?
				#-> delta between deformation and oscillation
				self.transform.rotate(angle=-self.rotation_absolute_delta[i][5,2]+self.rotation_absolute_delta[i][npp-2,2], rot_point = self.mbdyn_mesh[i][npp-2,:])
				self.displacement_relative_delta.append((self.transform.output() - self.mbdyn_mesh[i][:npp-2,:]))
				
			self.write_data.clear()
			for i in range(self.patches):
				#self.write_data.append(np.concatenate(np.array((self.displacement[i][:11,:],self.displacement[i][:11,:]))))
				#transform coordinate system (because of the farfield rotation)
				self.transform.importGrid(np.concatenate(np.array((self.displacement_relative_delta[i][:11,:],self.displacement_relative_delta[i][:11,:]))))				
				self.write_data.append(self.transform.rotate(angle=self.rotation_absolute_delta[i][5,2]).T)
				
				#No preCICE
				#self.write_data.append(np.array((XA.flatten()*0,YA.flatten()*0,ZA.flatten()*0)).T)
			
			if interface.is_write_data_required(dt):
				for i in range(self.patches):
					interface.write_block_vector_data(self.write_data_id[i], self.vertex_ids[i], self.write_data[i])
			
			print("DUMMY: Advancing in time")
			dt = interface.advance(dt)
			self.current_time_step += decimal.Decimal(str(dt))
			if interface.is_action_required(
				    precice.action_read_iteration_checkpoint()):
				print("DUMMY: Reading iteration checkpoint")
				interface.mark_action_fulfilled(
				    precice.action_read_iteration_checkpoint())
				exit("ERROR MBDyn got no values")
			else:
				#previous displacement = displacement.copy()
				
				#MBDyn advance
				if self.mbdyn.solve(True):
					print("diverged")
					break

			self.iteration += 1
		print("preCICE finalizing")
		interface.finalize()
		print("DUMMY: Closing python solver dummy...")
