/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SNAPMAKER_EMERGENCY_STOP_H_
#define SNAPMAKER_EMERGENCY_STOP_H_

#include "module_base.h"
#include "can_host.h"

enum EmergencyStopState : uint8_t {
  EMERGENCY_STOP_ONLINE,
  EMERGENCY_STOP_OFFLINE,
  EMERGENCY_STOP_TRIGGER,
  // EMERGENCY_STOP_WAIT_TRIGGER,
  // EMERGENCY_STOP_INVALID,
  // EMERGENCY_STOP_WAIT_TOOLHEAD,
  // EMERGENCY_STOP_WAIT_RESET,

  EMERGENCY_STOP_FALLING_EDGE_TRIGGER,
  EMERGENCY_STOP_RISING_EDGE_TRIGGER,
  EMERGENCY_STOP_WAIT_TOOLHEAD,
  EMERGENCY_STOP_NO_ACTION,
  EMERGENCY_STOP_INVALID
};

class EmergencyStop: public ModuleBase {
  public:
    EmergencyStop(): ModuleBase(MODULE_DEVICE_ID_EMERGENCY_STOP) {
      state_  = EMERGENCY_STOP_OFFLINE;
      event_state_ = EMERGENCY_STOP_OFFLINE;
      mac_index_   = MODULE_MAC_INDEX_INVALID;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    bool IsOnline() { return (mac_index_ != MODULE_MAC_INDEX_INVALID); }
    bool IsTriggered() {return state_ == EMERGENCY_STOP_TRIGGER;}
    void Disable();
    void Enable();
    //void SetStatus(uint8_t status) { state_ = (status == 0) ? EMERGENCY_STOP_TRIGGER : EMERGENCY_STOP_ONLINE;}
    void SetStatus(uint8_t status) { 
      event_state_ = status ? EMERGENCY_STOP_RISING_EDGE_TRIGGER : EMERGENCY_STOP_FALLING_EDGE_TRIGGER;
    }
    void Process();

    void PollState();

    // callback for HMI events
    ErrCode ReportStatus();

    uint32_t mac(uint8_t sub_index = 0) { return canhost.mac(mac_index_); }

  private:
    void HandleDoorOpened();
    void HandleDoorClosed();


  private:
    uint8_t mac_index_;
    EmergencyStopState state_;

    uint8_t event_state_;
    bool    enabled_;
};


extern EmergencyStop emergency_stop;

#endif
