    #define _USE_MATH_DEFINES
    #include <cmath>
    #include <iostream>
    #include <cstdlib>

    #include "rec/robotino/com/all.h"
    #include "rec/core_lt/utils.h"
    #include "rec/core_lt/Timer.h"

    #define FORWARD_VELOCITY 300
    #define BACKWARD_VELOCITY -150
    #define ROTATION_VELOCITY 300
    #define MAX_DISTANCE 3000
    //#define IRSIGNAL_TO_ROTATE 0.6

    using namespace rec::robotino::com;

    class MyInfo : public Info
    {
    public:
            MyInfo()
            {
            }

            void infoReceivedEvent( const char* text )
            {
                    std::cout << text << std::endl;
            }
    };

    class MyCom : public Com
    {
    public:
            MyCom()
            {
            }

            void errorEvent( Error error, const char* errorString )
            {
                    std::cerr << "Error: " << errorString << std::endl;
            }

            void connectedEvent()
            {
                    std::cout << "Connected." << std::endl;
            }

            void connectionClosedEvent()
            {
                    std::cout << "Connection closed." << std::endl;
            }
    };
    MyInfo info;
    Odometry odometry;
    MyCom com;
    Motor motor1;
    Motor motor2;
    Motor motor3;
    OmniDrive omniDrive;
    Bumper bumper;
    DistanceSensor distance0;
    DistanceSensor distance2;
    DistanceSensor distance7;

    float IRSIGNAL_TO_ROTATE = 0.6;
    float IRSIGNAL_TO_STOP = 0.7;
    int isInTheLine = 0;

    void init( const std::string& hostname )
    {
            odometry.setComId( com.id() );
            info.setComId( com.id() );

            // Inicializacao dos atuadores e sensores
            motor1.setComId( com.id() );
            motor1.setMotorNumber( 0 );

            motor2.setComId( com.id() );
            motor2.setMotorNumber( 1 );

            motor3.setComId( com.id() );
            motor3.setMotorNumber( 2 );

            omniDrive.setComId( com.id() );

            bumper.setComId( com.id() );

            distance0.setComId( com.id() );
            distance0.setSensorNumber( 0 );

            distance2.setComId( com.id() );
            distance2.setSensorNumber( 2 );

            distance7.setComId( com.id() );
            distance7.setSensorNumber( 7 );

            // Coneco com o robotino
            std::cout << "Connecting..." << std::endl;
            com.setAddress( hostname.c_str() );
            odometry.set( 0, 0, 0 );
            com.connect();

            std::cout << std::endl << "Connected" << std::endl;
    }

    void destroy()
    {
        com.disconnect();
    }

    void move_forward()
    {
        while (distance0.voltage() < IRSIGNAL_TO_ROTATE)        // sem obstáculo à frente
        {
            omniDrive.setVelocity( FORWARD_VELOCITY, 0, 0 );    // vai para frente
            if (odometry.x() >= MAX_DISTANCE)                   // chegou no ponto final
            {
                destroy();                                      // pára o robô
            }
        }
    }

    void contorna()
    {
        while(true)
        {
            std::cout << "isInTheLine: " << isInTheLine << std::endl;
            while( distance0.voltage() > IRSIGNAL_TO_ROTATE)    // com obstáculo à frente
            {
                omniDrive.setVelocity( 0, 130, 0 );             // vai para a esquerda
                if (odometry.y() < -10)      // o robô está na linha reta
                {
                    isInTheLine = 1;
                    std::cout << "11 "<< std::endl;
                    break;
                }
            }
            if(isInTheLine == 1)
            {
                std::cout << "22 "<< std::endl;
                break;
            }
            while( distance0.voltage() < IRSIGNAL_TO_ROTATE)    // sem obstáculo à frente
            {
                omniDrive.setVelocity( 0, 40, -50 );            // gira no sentido horário
                if (odometry.y() < -10)      // o robô está na linha reta
                {
                    isInTheLine = 1;
                    std::cout << "33 "<< std::endl;
                    break;
                }
            }
            if(isInTheLine == 1)
            {
                break;
            }
        }
    }

    void gira180()
    {
        isInTheLine = 0;
        odometry.set( odometry.x(), odometry.y(), -179 );
        while (odometry.phi() < -1)
        {
            std::cout << "X: " << odometry.x() << "  Y: " << odometry.y() << "  Theta: " << odometry.phi() << std::endl;
            omniDrive.setVelocity(0,0,40);  // gira no sentido anti-horário
        }
    }

    void drive()
    {
            rec::core_lt::Timer timer;
            timer.start();
            rec::iocontrol::remotestate::SensorState sensorState;

    while( com.isConnected() && sensorState.bumper == false)
    {
        std::cout << "X: " << odometry.x() << "  Y: " << odometry.y() << "  Theta: " << odometry.phi() << std::endl;

    move_forward();

    if (distance0.voltage() > IRSIGNAL_TO_ROTATE)   // identificou obstáculo
    {
        contorna();
    }

    if (isInTheLine == 1)                           // está na linha reta
    {
        gira180();                                  // gira 180 graus
    }

    }
}

    int main( int argc, char **argv )
    {
            std::string hostname = "172.26.1.1";
            if( argc > 1 )
            {
                    hostname = argv[1];
            }

            try
            {
                    init( hostname );
                    drive();
                    destroy();
            }
            catch( const rec::robotino::com::ComException& e )

            {
                    std::cerr << "Com Error: " << e.what() << std::endl;
            }
            catch( const std::exception& e )
            {
                    std::cerr << "Error: " << e.what() << std::endl;
            }
            catch( ... )
            {
                    std::cerr << "Unknow Error" << std::endl;
            }
    }
