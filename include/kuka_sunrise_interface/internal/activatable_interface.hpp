/*
 * activatable_interface.hpp
 *
 *  Created on: Nov 8, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERFACE_INTERNAL_ACTIVATABLE_INTERFACE_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERFACE_INTERNAL_ACTIVATABLE_INTERFACE_HPP_

namespace kuka_sunrise_interface{

class ActivatableInterface{
public:
  virtual bool activate(){
    is_active_ = true;
    return true;
  };
  virtual bool deactivate(){
    is_active_ = false;
    return true;
  }
  bool isActive(){
    return is_active_;
  }
  virtual ~ActivatableInterface();
protected:
  bool is_active_;
};


}



#endif /* INCLUDE_KUKA_SUNRISE_INTERFACE_INTERNAL_ACTIVATABLE_INTERFACE_HPP_ */
