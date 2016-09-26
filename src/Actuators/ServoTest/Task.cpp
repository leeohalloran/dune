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
// Author: Joao_teixeira                                                    *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Math/Constants.hpp>

namespace Actuators
{
  namespace ServoTest
  {
    using DUNE_NAMESPACES;

      //! Number of fins.
    static const int c_fins = 4;

    //! Task arguments.
    struct Arguments
    {
    //test duration in minutes
    double duration;
    //! miliseconds por cycle
    double cycle;
    //servo index
    std::string index;
    //state machine
    bool sm_state;
    };


    struct Task: public DUNE::Tasks::Task
    {

      //! Task arguments.
      Arguments m_args;

      IMC::SetServoPosition m_fins[c_fins];
      //! Time Delta
      Time::Delta m_delta[c_fins];
      //! Timer for timeout of
      Time::Counter<float> m_timer;

      Time::Counter<float> m_cycle;

      float m_servo_pos[c_fins];
      // task active
      bool m_sm_state;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_sm_state(false)
      {
        param("Time of test minutes", m_args.duration)
        .defaultValue("1")
        .description("Time of test duration (minutes)");

        param("Cycle in seconds", m_args.cycle)
        .defaultValue("1")
        .description("Cycle duration of actuation miliseconds");

        param("Servo index", m_args.index)
        .defaultValue("All")
        .description("Servo index or test all");

        param("Start_Stop", m_args.sm_state)
        .defaultValue("false")
        .description("Start / Stop teste");


      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {

        m_sm_state=m_args.sm_state;
        if (m_sm_state==true)
        {
          spew("servo motor test start to run - swep mesg");
          m_timer.setTop(m_args.duration*60);
          m_cycle.setTop(m_args.cycle);
        }
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        // Initialize fin commands.
        for (int i = 0; i < c_fins; i++)
          {
            m_fins[i].id = i;
            m_fins[i].value= (float) ((Math::c_pi) / 2);
            m_servo_pos[i] = 0.0;
          }
      }

      inline void
      dispatchFins(void)
      {

        int i=0;
        if ( m_args.index.compare("All") == 0)
        {
          for (i = 0; i < c_fins; i++)
          {
            m_fins[i].value = -(m_fins[i].value);
            dispatch(m_fins[i]);
            spew("change position %d",i);
          }
        }
        else
        {
          i=std::atoi(m_args.index.c_str());
          if(i>=0 && i<4)
          {
            m_fins[i].value = -(m_fins[i].value);
            dispatch(m_fins[i]);
            spew("change position %d",i);
          }
        }
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      //! Main loop.
      void
      onMain(void)
      {

        m_timer.setTop(m_args.duration*60);
        m_cycle.setTop(m_args.cycle);

        while (!stopping())
        {
          if(m_sm_state==true)
          {
            if(! m_timer.overflow())
            {
              if(m_cycle.overflow())
              {
                m_cycle.reset();
                dispatchFins();
              }
            }
            else
            {
              m_sm_state=false;
              spew("The End");
            }
          }
          waitForMessages(0.1);
        }
      }
    };
  }
}

DUNE_TASK
