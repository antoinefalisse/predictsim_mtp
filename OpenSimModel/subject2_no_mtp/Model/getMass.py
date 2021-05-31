# -*- coding: utf-8 -*-
"""
Created on Mon May 31 10:46:08 2021

@author: u0101727
"""

import opensim

scale_factor = 0.9
nameModel = "subject2_withoutMTP_weldRadius_scaled_FK_contactsAsForces_generic.osim"
name_newModel = nameModel[:-5] + "_lowerBody" + str(scale_factor).replace('.', '') + ".osim"

model = opensim.Model(nameModel)
bodySet = model.getBodySet()


for idxBody in range(bodySet.getSize()):
    
    c_body = bodySet.get(idxBody)
    c_bodyName = c_body.getName()
    # print(c_bodyName)
    
    if c_bodyName == "torso":
        continue
    
    mass = c_body.get_mass()    
    new_mass = mass * scale_factor
    c_body.set_mass(new_mass)
    
    inertia = c_body.get_inertia()   
    # inertia_str = inertia.toString()
    new_inertia = inertia.scalarTimesEq(scale_factor)    
    # new_inertia_str = new_inertia.toString()
    c_body.set_inertia(new_inertia)
    
    
    
# model.printToXML(name_newModel)

totalmass = 0
for idxBody in range(bodySet.getSize()):
    c_body = bodySet.get(idxBody)
    mass = c_body.get_mass() 
    totalmass += mass
    