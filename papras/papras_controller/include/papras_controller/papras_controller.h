#ifndef PAPRAS_CONTROLLER_H
#define PAPRAS_CONTROLLER_H

#include "../../open_manipulator_p/open_manipulator_p_controller/include/open_manipulator_p_controller/open_manipulator_p_controller.h"

class PaprasController : public OpenManipulatorController
{
      public:
        PaprasController();
        virtual ~PaprasController();
};
#endif // PAPRAS CONTROLLER
