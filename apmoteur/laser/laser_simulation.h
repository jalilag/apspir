//indicateur de fonctionnement du module
extern int laser_simu;

//perturbations
void laser_simulation_inc_const_err(INTEGER32 val);
void laser_simulation_dec_const_err(INTEGER32 val);

void laser_simulation_inc_vel_err(INTEGER32 val);
void laser_simulation_dec_vel_err(INTEGER32 val);

INTEGER32 laser_simulation_get_const_err(void);
INTEGER32 laser_simulation_get_vel_err(void);




//donnée à fournir par l'utilisateur
extern laser lsim;

//fonction utilisateur
void * laser_simulation_SimuThread_func(void * arg);
void laser_simulation_on_RALENTIR_clicked(void);
void laser_simulation_on_ACCELERER_clicked(void);




