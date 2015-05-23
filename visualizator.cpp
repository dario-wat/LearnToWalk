#include "visualizator.hpp"

Visualizator::Visualizator(Robot* robot, ANN* ann, dWorldID world, dSpaceID space, double sim_step)
	: robot_(robot), ann_(ann), world_(world), space_(space), sim_step_(sim_step) {

	data.world_ = world_;
	// data.contact_group_ = contact_group_;
	data.contact = &contact[0];
	data.size = CONTACT_ARR_SIZE;
}

void Visualizator::nearCallback(void *data, dGeomID o1, dGeomID o2) {
	CallbackData* dat = (CallbackData*) data;

	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
		return;
	}
	
	int n = dCollide(o1, o2, dat->size, &dat->contact[0].geom, sizeof(dContact));
	// for (int i = 0; i < n; i++) {
	// 	dat->contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
	// 	dat->contact[i].surface.mu = 100.0; 
	// 	dat->contact[i].surface.soft_erp = ERP_P;//(one_step*para_K)/(one_step*para_K + para_C);		//ERP
	// 	dat->contact[i].surface.soft_cfm = CFM_P;//1.0/(one_step*para_K+para_C);		//CFM
	// 	dJointID c = dJointCreateContact(dat->world_, dat->contact_group_, &dat->contact[i]);
	// 	dJointAttach(c, b1, b2);
	// }
}

void Visualizator::simLoop() {
	dSpaceCollide(space_, &data, &nearCallback);
	dWorldStep(world_, sim_step_);
	ann_->feedThrough(&input[0], &new_state[0][0]);
	robot_->setNewState(new_state);
	robot_->walk();
	robot_->draw();
}