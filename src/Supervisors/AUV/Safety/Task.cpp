//***************************************************************************
// Copyright 2007-2016 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Pedro Calado                                                     *
// Author: José Braga                                                       *
//***************************************************************************

// ISO C++ 98 headers.
#include <cstring>
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Supervisors
{
  namespace AUV
  {
    //! This supervisor ensures that the system remains stationary while idle
    //! at surface, or, in case of error and lack of consoles in the network,
    //! that it is able to go to a safety location.
    namespace Safety
    {
      using DUNE_NAMESPACES;

      //! Time to wait to check conditions after failed attempt.
      static const float c_fail_timeout = 60.0f;
      //! Distance to safety position threshold.
      static const float c_safety_dist = 50.0f;

      //! %Task arguments.
      struct Arguments
      {
        //! Heartbeat timeout.
        float timeout;
        //! Keep station.
        bool sk;
        //! Ascend with popup.
        bool asc;
      };

      //! Data available.
      enum DataReady
      {
        //! Got no data
        GOT_NOTHING = 0x00,
        //! Got vehicle state
        GOT_VSTATE = 0x01,
        //! Got vehicle medium
        GOT_MEDIUM = 0x02,
        //! Got Plan control state
        GOT_PCS = 0x04,
        //! Got all
        GOT_ALL = 0x07
      };

      struct Task: public DUNE::Tasks::Periodic
      {
        //! Lost communications timer.
        Counter<double> m_lost_coms_timer;
        //! Timer to wait after failing again.
        Counter<float> m_fail_timer;
        //! Medium handler.
        Monitors::MediumHandler m_medium;
        //! Vehicle state is error or service
        bool m_serv_err;
        //! Plan Control State
        IMC::PlanControlState m_pcs;
        //! Plan Generation request.
        IMC::PlanGeneration m_pg;
        //! Availability of data
        unsigned m_dr;
        //! Is system near safety position.
        bool m_is_near;
        //! Issued plan.
        bool m_issued;
        //! Issued go to safety.
        bool m_safety;
        //! Task arguments.
        Arguments m_args;

        Task(const std::string& name, Tasks::Context& ctx):
          Tasks::Periodic(name, ctx),
          m_serv_err(false),
          m_dr(GOT_NOTHING),
          m_is_near(true),
          m_issued(false),
          m_safety(false)
        {
          param("Lost Comms Timeout", m_args.timeout)
          .defaultValue("600")
          .minimumValue("60")
          .units(Units::Second)
          .description("Timeout before sending system to safety zone, if defined");

          param("Keep Station At Surface", m_args.sk)
          .defaultValue("true")
          .description("Once vehicles pops at surface, keep station");

          param("Ascend With Actuation", m_args.asc)
          .defaultValue("true")
          .description("Let natural buoyancy elevate the vehicle or execute a popup");

          bind<IMC::EstimatedState>(this);
          bind<IMC::Heartbeat>(this);
          bind<IMC::PlanControl>(this);
          bind<IMC::PlanControlState>(this);
          bind<IMC::VehicleState>(this);
          bind<IMC::VehicleMedium>(this);
        }

        void
        onUpdateParameters(void)
        {
          if (paramChanged(m_args.timeout))
            m_lost_coms_timer.setTop(m_args.timeout);
        }

        void
        onResourceInitialization(void)
        {
          m_pg.op = IMC::PlanGeneration::OP_REQUEST;
          m_pg.cmd = IMC::PlanGeneration::CMD_EXECUTE;
          m_pg.params = "calibrate=false;ignore_errors=true";

          m_fail_timer.setTop(c_fail_timeout);

          // Initialize entity state.
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
        }

        void
        consume(const IMC::EstimatedState* msg)
        {
          double lat, lon;
          Coordinates::toWGS84(*msg, lat, lon);

          double rlat = 0.0;
          double rlon = 0.0;
          double dist;

          dist = Coordinates::WGS84::distance(lat, lon, 0.0, rlat, rlon, 0.0);

          if (dist > c_safety_dist)
            m_is_near = false;
          else
            m_is_near = true;
        }

        void
        consume(const IMC::Heartbeat* msg)
        {
          if (msg->getSource() == getSystemId())
            return;

          if ((msg->getSource() & 0x4000) == 0)
            return;

          m_lost_coms_timer.reset();
        }

        void
        consume(const IMC::PlanControl* msg)
        {
          // other system is managing this one
          if (msg->getSource() != getSystemId())
          {
            m_issued = false;
            m_safety = false;
          }
        }

        void
        consume(const IMC::PlanControlState* msg)
        {
          m_dr |= GOT_PCS;
          m_pcs = *msg;
        }

        void
        consume(const IMC::VehicleState* msg)
        {
          if (msg->op_mode != IMC::VehicleState::VS_BOOT)
            m_dr |= GOT_VSTATE;

          m_serv_err = ((msg->op_mode == IMC::VehicleState::VS_SERVICE) ||
                        (msg->op_mode == IMC::VehicleState::VS_ERROR));
        }

        void
        consume(const IMC::VehicleMedium* msg)
        {
          m_dr |= GOT_MEDIUM;
          m_medium.update(msg);

          if (m_medium.isUnderwater())
            m_lost_coms_timer.reset();
        }

        //! Checks if the vehicle is plan ready or blocked.
        //! @return true if conditions are met, false otherwise.
        bool
        isIdle(void)
        {
          return ((m_pcs.state == IMC::PlanControlState::PCS_BLOCKED) ||
                  (m_pcs.state == IMC::PlanControlState::PCS_READY));
        }

        //! Checks if the vehicle last outcome is failure.
        //! @return true if conditions are met, false otherwise.
        bool
        hasFailed(void)
        {
          return m_pcs.last_outcome == IMC::PlanControlState::LPO_FAILURE;
        }

        //! Check if vehicle is near safety position.
        //! @return true if safety is nearby, or simply not defined.
        bool
        isNear(void)
        {
          // @todo return true if not defined (anywhere is safe?)
          return m_is_near;
        }

        //! Check if we need to move the vehicle to safe zone.
        //! @return true if system is safe, false otherwise.
        bool
        isSafe(void)
        {
          // near safe zone already, or, safe zone is not defined
          if (isNear())
            return true;

          // already in safety mode
          if (m_safety)
            return true;

          // we have heartbeat to console
          if (!m_lost_coms_timer.overflow())
            return true;

          // vehicle state is in service or error, and plan control
          // state is idle, so system may be drifting
          if (m_serv_err && isIdle())
            return false;

          // although executing, it's running a self imposed plan
          if (m_issued)
            return false;

          return true;
        }

        //! Send system to safety.
        void
        goToSafety(void)
        {
          m_safety = true;
          m_pg.plan_id = "safety_zone";
          dispatch(m_pg);
        }

        void
        task(void)
        {
          if (m_dr != GOT_ALL)
            return;

          // don't actuate out of water
          if (!m_medium.inWater())
            return;

          // system not safe, send to safe zone
          if (!isSafe())
          {
            m_lost_coms_timer.reset();
            goToSafety();
            return;
          }

          // do not keep station or ascend.
          if (!m_args.sk && !m_args.asc)
            return;

          // in service or error
          if (!m_serv_err)
            return;

          // not idle
          if (!isIdle())
            return;

          m_pg.plan_id.clear();

          // last outcome is failure
          if (hasFailed())
          {
            if (!m_fail_timer.overflow())
              return;

            m_fail_timer.reset();
            m_pg.plan_id = "safety_";
          }

          if (m_medium.isUnderwater())
          {
            if (!m_args.asc)
              return;

            m_pg.plan_id += "surface";
          }
          else
          {
            if (!m_args.sk)
              return;

            m_pg.plan_id += "sk";
          }

          m_issued = true;
          dispatch(m_pg);
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }
      };
    }
  }
}

DUNE_TASK
