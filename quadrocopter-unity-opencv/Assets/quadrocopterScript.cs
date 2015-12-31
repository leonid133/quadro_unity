using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using ClassLibraryNeuralNetworks;


public class quadrocopterScript : MonoBehaviour
{
    //нейронная сеть

    public double Kerr;
    public double Klern = 0.05;

    public double net_roll;
    public double net_yaw;
    public double net_pich;

    // регулировки высоты
    private static int[] Layers = { 100, 1 };
    private static NeuralNW m_Net_H = new NeuralNW(4, Layers);
    
    public bool H_neyro_on = false;
    public double thrLimit;

    // регулировка roll pich
    public bool neyro_gps_on = false;

    private static int[] Layers_Net_Roll = { 200, 1 };
    private static NeuralNW m_Net_roll = new NeuralNW(4, Layers_Net_Roll);

    private static int[] Layers_Net_Pich = { 200, 1 };
    private static NeuralNW m_Net_pich = new NeuralNW(4, Layers_Net_Pich);


    //axel autopulot

    public double H_;
    public double geo_X;
    public double geo_Z;

    public double Ax;
    public double Ay;
    public double Az;

    public double targetH;
    public double targetX;
    public double targetZ;

    //PID регуляторы которые будут стабилизировать высоту и координаты
    //private PID H_PID = new PID(80, 2, 50);
    //private PID H_PID = new PID(40, 2, 50);
    private PID H_PID = new PID(20, 0.01, 30);

    // private PID X_PID = new PID(1, 0.000005, 30);
    // private PID Z_PID = new PID(1, 0.000005, 30);

    /*
     private PID X_PID = new PID(2, 0.000005, 5);
     private PID Z_PID = new PID(2, 0.000005, 5);
     */
    /*
    private PID X_PID = new PID(1, 0.001, 2);
    private PID Z_PID = new PID(1, 0.001, 2);*/

    private PID X_PID = new PID(0.8, 0.0000128, 4);
    private PID Z_PID = new PID(0.8, 0.0000128, 4);
    
    public double ErrAccel;
    public double ErrGPS;

    //фактические параметры
    private double pitch; //Тангаж
	private double roll; //Крен
	private double yaw; //Рыскание
	public double throttle; //Газ, газ мы задаем извне, поэтому он public

	//требуемые параметры
	public double targetPitch;
	public double targetRoll;
	public double targetYaw;

    //PID регуляторы, которые будут стабилизировать углы

    private PID pitchPID = new PID (100, 0, 20); //держится на ошибке 7 м/с2 но очень беспокойно себя ведет
    private PID rollPID = new PID (100, 0, 20);

    /*
    private PID pitchPID = new PID(20, 1, 15); //держится на ошибке 4 м/с2
    private PID rollPID = new PID(20, 1, 15);
    */
    /*
       private PID pitchPID = new PID(10, 1, 1); //плавно держится на 7 но медленно реагирует на события, может перевернуться 
       private PID rollPID = new PID(10, 1, 1);
      */
    //private PID pitchPID = new PID(10, 0, 2); // 5 м/с2 держится плавно реагирует на ошибки
    //private PID rollPID = new PID(10, 0, 2);

    //private PID pitchPID = new PID(20, 0.001, 50); 
    //private PID rollPID = new PID(20, 0.001, 50);

    private PID yawPID = new PID (50, 0, 50);

	//private Quaternion prevRotation = new Quaternion (0, 1, 0, 0);

    //mouse contol
    float x = 0.0f;
    float y = 0.0f;
    float throttleSpeed = 0.01f;
    float YawSpeed = 2.5f;
    float RollSpeed = 2.5f;
    float PitchSpeed = 2.5f;


    //автотест
    private double lasttime = 0.0f;
    private double d_time = 0.0f;
    public bool autotest_on = true;

    void readControl() {

        if(autotest_on)
        {
            d_time +=  Convert.ToDouble(Time.time) - lasttime;
            lasttime = Convert.ToDouble(Time.time);
            if(d_time > 60)
            {
                d_time = 0.0f;

                System.Random rnd = new System.Random();
                double rnd_ = Convert.ToDouble(rnd.Next(-1000, 1000)) * 0.001f * 50;
                targetX = rnd_;

                rnd = new System.Random();
                rnd_ = Convert.ToDouble(rnd.Next(-1000, 1000)) * 0.001f * 50;
                targetZ = rnd_;
                /*
                rnd = new System.Random();
                rnd_ = Convert.ToDouble(rnd.Next(0, 1000)) * 0.001f * 360;
                targetYaw = rnd_;
                */
                rnd = new System.Random();
                rnd_ = 30 + Convert.ToDouble(rnd.Next(0, 1000)) * 0.001f * 10;
                targetH = rnd_;
            }
        }
        /*
        throttle += Input.GetAxis("Vertical")* throttleSpeed;
        targetYaw += Input.GetAxis("Horizontal")* YawSpeed;
        if (Input.GetMouseButton(0))
        {
            
            targetRoll += (Input.GetAxis("Mouse X")) * RollSpeed;
            targetPitch -= (Input.GetAxis("Mouse Y")) * PitchSpeed;
        }
        
        x = Input.GetAxis("Mouse X");
        y = Input.GetAxis("Mouse Y");
        if(Input.GetKey("q"))
        { targetRoll += 1; }
        if(Input.GetKey("e"))
        { targetRoll -= 1; }
        if (Input.GetKey("z"))
        { targetPitch += 1; }
        if (Input.GetKey("c"))
        { targetPitch -= 1; }
        */
        if (Input.GetKey("left ctrl"))
        {
            //throttle = 0;
            targetRoll = 0;
            targetPitch = 0;
            targetYaw = 0;
        }
        if (Input.GetKey("1"))
        {
            //throttle = 10;
            targetH = 5;
            targetX = 0;
            targetZ = 0;
        }
        if (Input.GetKey("2"))
        {
            //throttle = 20;
            targetH = 5;
            targetX = 10;
            targetZ = 0;
        }
        if (Input.GetKey("3"))
        {
            //throttle = 30;
            targetH = 5;
            targetX = -10;
            targetZ = 0;
        }
        if (Input.GetKey("4"))
        {
            //throttle = 40;
            targetH = 5;
            targetX = -10;
            targetZ = 10;
        }
        if (Input.GetKey("5"))
        {
            //throttle = 50;
            targetH = 5;
            targetX = -10;
            targetZ = -10;
        }
    }

    private static Vector3[] positionRegister;
    private static float[] posTimeRegister;
    private static int positionSamplesTaken = 0;
    public static bool LinearAcceleration(out Vector3 vector, Vector3 position, int samples)
    {

        Vector3 averageSpeedChange = Vector3.zero;
        vector = Vector3.zero;
        Vector3 deltaDistance;
        float deltaTime;
        Vector3 speedA;
        Vector3 speedB;

        if (samples < 3)
        {
            samples = 3;
        }

        //Initialize
        if (positionRegister == null)
        {
            positionRegister = new Vector3[samples];
            posTimeRegister = new float[samples];
        }
        
        for (int i = 0; i < positionRegister.Length - 1; i++)
        {
            positionRegister[i] = positionRegister[i + 1];
            posTimeRegister[i] = posTimeRegister[i + 1];
        }
        positionRegister[positionRegister.Length - 1] = position;
        posTimeRegister[posTimeRegister.Length - 1] = Time.time;
        positionSamplesTaken++;

        if (positionSamplesTaken >= samples)
        {
            for (int i = 0; i < positionRegister.Length - 2; i++)
            {
                deltaDistance = positionRegister[i + 1] - positionRegister[i];
                deltaTime = posTimeRegister[i + 1] - posTimeRegister[i];
               
                if (deltaTime == 0)
                {
                    return false;
                }

                speedA = deltaDistance / deltaTime;
                deltaDistance = positionRegister[i + 2] - positionRegister[i + 1];
                deltaTime = posTimeRegister[i + 2] - posTimeRegister[i + 1];

                if (deltaTime == 0)
                {

                    return false;
                }
                speedB = deltaDistance / deltaTime;
                averageSpeedChange += speedB - speedA;
            }

            averageSpeedChange /= positionRegister.Length - 2;
            float deltaTimeTotal = posTimeRegister[posTimeRegister.Length - 1] - posTimeRegister[0];

            vector = averageSpeedChange / deltaTimeTotal;
            return true;
        }
        else
        {
            return false;
        }
    }

  
    private Vector3 lastVelocity;
    private float sensor_timer;
    private Vector3 acceleration_save;
    private float AY_neyr;
    //получаем показания акселерометров
    public Vector3 GetSensors()
    {
        sensor_timer += Time.deltaTime;
        Vector3 acceleration;
        if (sensor_timer > 0.005) //200Гц
        {
            Vector3 rot = GameObject.Find("Sensors").GetComponent<Transform>().rotation.eulerAngles;
            Vector3 pos = GameObject.Find("Sensors").GetComponent<Transform>().position;

            //поворачиваем гравитацию на угол поворота сенсора, 
            //чтобы получить значения сходные с реальными показаниями акселей
            Vector3 grav = Physics.gravity;
            Vector3 newgrav = Vector3.down;

            newgrav.x = (float)(grav.x * Math.Cos(-Math.PI * rot.y / 180.0f) + grav.z * Math.Sin(-Math.PI * rot.y / 180.0f));
            newgrav.y = grav.y;
            newgrav.z = -(float)(grav.x * Math.Sin(-Math.PI * rot.y / 180.0f) + grav.z * Math.Cos(-Math.PI * rot.y / 180.0f));
            grav = newgrav;

            newgrav.x = grav.x;
            newgrav.y = (float)(grav.y * Math.Cos(Math.PI * rot.x / 180.0f) - grav.z * Math.Sin(Math.PI * rot.x / 180.0f));
            newgrav.z = (float)(grav.y * Math.Sin(Math.PI * rot.x / 180.0f) + grav.z * Math.Cos(Math.PI * rot.x / 180.0f));
            grav = newgrav;

            newgrav.x = (float)(grav.x * Math.Cos(Math.PI * rot.z / 180.0f) - grav.y * Math.Sin(Math.PI * rot.z / 180.0f));
            newgrav.y = (float)(grav.x * Math.Sin(Math.PI * rot.z / 180.0f) + grav.y * Math.Cos(Math.PI * rot.z / 180.0f));
            newgrav.z = grav.z;
            grav = newgrav;

            //измеряем ускорение через изменение скорости
            
            LinearAcceleration(out acceleration, pos, 50);
            AY_neyr = acceleration.y;
            lastVelocity = GameObject.Find("Frame").GetComponent<Rigidbody>().velocity;
            acceleration += grav; //добавляем ускорение свободного падения

            // добавляем ошибку
            System.Random x_rnd = new System.Random();
            double x_err = Convert.ToDouble(x_rnd.Next(-1000, 1000)) * 0.001f * ErrAccel;
            acceleration.x += (float)x_err;

            System.Random y_rnd = new System.Random();
            double y_err = Convert.ToDouble(y_rnd.Next(-1000, 1000)) * 0.001f * ErrAccel;
            acceleration.y += (float)y_err;

            System.Random z_rnd = new System.Random();
            double z_err = Convert.ToDouble(z_rnd.Next(-1000, 1000)) * 0.001f * ErrAccel;
            acceleration.z += (float)z_err;

            acceleration_save = acceleration;
            sensor_timer = 0;
        }
        else
        {
            acceleration = acceleration_save;
        }
        return acceleration;
    }

    private Vector3 gps_save;
    private float gps_timer;
    private bool GetGPS(out Vector3 gps_XHZ)
    {
        gps_timer += Time.deltaTime;
        Vector3 pos = GameObject.Find("Sensors").GetComponent<Transform>().position;
        if (gps_timer > 1) //1Гц
        {
            System.Random X_rnd = new System.Random();
            double X_err = Convert.ToDouble(X_rnd.Next(-1000, 1000)) * 0.001f * ErrGPS;
            gps_XHZ.x = pos.x + (float)X_err;
            gps_XHZ.y = pos.y;

            System.Random Z_rnd = new System.Random();
            double Z_err = Convert.ToDouble(Z_rnd.Next(-1000, 1000)) * 0.001f * ErrGPS;
            gps_XHZ.z = pos.z + (float)Z_err;
            
            gps_save = gps_XHZ;
            gps_timer = 0.0f;

            //Neyron
            double dgeoX = targetX - gps_XHZ.x;
            dgeoX = dgeoX < -50.0 ? -50.0 : dgeoX;
            dgeoX = dgeoX > 50.0 ? 50.0 : dgeoX;
            dgeoX /= 100.0;
            m_X_roll[0] = m_X_roll[1];
            m_X_roll[1] = m_X_roll[2];
            m_X_roll[2] = dgeoX;

            double dgeoZ = targetZ - gps_XHZ.z;
            dgeoZ = dgeoZ < -50.0 ? -50.0 : dgeoZ;
            dgeoZ = dgeoZ > 50.0 ? 50.0 : dgeoZ;
            dgeoZ = dgeoZ / 100.0;
            m_X_pich[0] = m_X_pich[1];
            m_X_pich[1] = m_X_pich[2];
            m_X_pich[2] = dgeoZ;

            return true;
        }
        else
        {
            gps_XHZ = gps_save;
            gps_XHZ.y = pos.y; // Высота у нас с барометра берется, и не ограничена одной посылкой в секунду
            return false;
        }
    }
    double[] deltaH = new double[4];
    private float h_timer;
    private double h_save;
    private void GetH(out double h)
    {
        h_timer += Time.deltaTime;
        Vector3 pos = GameObject.Find("Sensors").GetComponent<Transform>().position;
        if (h_timer > 0.005) //200Гц
        {
            //H Neyron
            deltaH[3] = AY_neyr;
           h_save = h = pos.y;
            deltaH[2] = deltaH[1];
            deltaH[1] = deltaH[0];
            deltaH[0] = (targetH - H_);
            deltaH[0] = deltaH[0] < -50 ? -50 : deltaH[0];
            deltaH[0] = deltaH[0] > 50 ? 50 : deltaH[0];
            deltaH[0] /= 100.0;
            h_timer = 0.0f;

        }
        else
        {
            h = h_save;
        }
    }
    private float netsave_timer;
    private void NetSave()
    {
        netsave_timer += Time.deltaTime;
        if(netsave_timer > 60)
        {
            m_Net_H.SaveNW("net_h.txt");
            m_Net_pich.SaveNW("net_pich.txt");
            m_Net_roll.SaveNW("net_roll.txt");
            netsave_timer = 0.0f;
        }
    }

    class KalmanFilterSimple1D
    {
        public double X0 { get; private set; } // predicted state
        public double P0 { get; private set; } // predicted covariance

        public double F { get; private set; } // factor of real value to previous real value
        public double Q { get; private set; } // measurement noise
        public double H { get; private set; } // factor of measured value to real value
        public double R { get; private set; } // environment noise

        public double State { get; private set; }
        public double Covariance { get; private set; }

        public KalmanFilterSimple1D(double q, double r, double f = 1, double h = 1)
        {
            Q = q;
            R = r;
            F = f;
            H = h;
        }

        public void SetState(double state, double covariance)
        {
            State = state;
            Covariance = covariance;
        }

        public void Correct(double data)
        {
            //time update - prediction
            X0 = F * State;
            P0 = F * Covariance * F + Q;

            //measurement update - correction
            var K = H * P0 / (H * P0 * H + R);
            State = X0 + K * (data - H * X0);
            Covariance = (1 - K * H) * P0;
        }
    }
    private static KalmanFilterSimple1D kalman_x = new KalmanFilterSimple1D(f: 1, h: 1, q: 500.8, r: 1050.85); // задаем F, H, Q и R
    private static KalmanFilterSimple1D kalman_y = new KalmanFilterSimple1D(f: 1, h: 1, q: 500.8, r: 1050.85);
    private static KalmanFilterSimple1D kalman_z = new KalmanFilterSimple1D(f: 1, h: 1, q: 500.8, r: 1050.85);

    private static KalmanFilterSimple1D kalman_gps_x = new KalmanFilterSimple1D(f: 1.0, h: 1.0, q: 5.0, r: 10.0); 
    private static KalmanFilterSimple1D kalman_gps_z = new KalmanFilterSimple1D(f: 1.0, h: 1.0, q: 5.0, r: 10.0);

    private static KalmanFilterSimple1D kalman_t1 = new KalmanFilterSimple1D(f: 1.1, h: 0.9, q: 15.0, r: 15.0);
    private static KalmanFilterSimple1D kalman_t2 = new KalmanFilterSimple1D(f: 1.1, h: 0.9, q: 15.0, r: 15.0);
    private static KalmanFilterSimple1D kalman_t3 = new KalmanFilterSimple1D(f: 1.1, h: 0.9, q: 15.0, r: 15.0);
    private static KalmanFilterSimple1D kalman_t4 = new KalmanFilterSimple1D(f: 1.1, h: 0.9, q: 15.0, r: 15.0);

    private  double[] m_X_roll = new double[4];
    private double[] m_X_pich = new double[4];

    void readRotation () {

        //фактическая ориентация квадрокоптера,
        //в реальном квадрокоптере эти данные необходимо получать
        //из акселерометра-гироскопа-магнетометра

        Vector3 rot = GameObject.Find ("Sensors").GetComponent<Transform> ().rotation.eulerAngles;
        Vector3 pos = GameObject.Find("Sensors").GetComponent<Transform> ().position;

        Vector3 acceleration = GetSensors();

        //выводим показания акселерометров
        Ax = acceleration.x;
        Ay = acceleration.y;
        Az = acceleration.z;

       // kalman.SetState(0.0, 0.1); // Задаем начальные значение State и Covariance
        kalman_x.Correct(acceleration.x);
        Ax = kalman_x.State;
        kalman_y.Correct(acceleration.y);
        Ay = kalman_y.State;
        kalman_z.Correct(acceleration.z);
        Az = kalman_z.State;

        //получаем углы на которые акселерометры по трем осям повернуты относительно земли
        Vector3 accel_rot = Vector3.zero;
        accel_rot.x = (float)(360.0f / 2 * Math.PI) * (float)Math.Atan(Ax / Math.Sqrt(Math.Pow(Ay, 2) + Math.Pow(Az, 2)));
        accel_rot.y = (float)(360.0f / 2 * Math.PI) * (float)Math.Atan(Ay / Math.Sqrt(Math.Pow(Ax, 2) + Math.Pow(Az, 2)));
        accel_rot.z = (float)(360.0f / 2 * Math.PI) * (float)Math.Atan(Az / Math.Sqrt(Math.Pow(Ax, 2) + Math.Pow(Ay, 2)));

        //поворачиваем вектор направленный в землю, на угол акселерометров, чтобы получить углы объекта относительно земли
        accel_rot = Quaternion.FromToRotation(Vector3.down, accel_rot).eulerAngles;

        //отображаем полученное с помощью акселерометров положение 
        GameObject.Find("Cube_girotest").GetComponent<Transform>().rotation = Quaternion.FromToRotation(GameObject.Find("Cube_girotest").GetComponent<Transform>().rotation.eulerAngles, Vector3.zero);
        GameObject.Find("Cube_girotest").GetComponent<Transform>().Rotate(accel_rot);
       
        Vector3 girotest_pos;
        
        girotest_pos.x = pos.x;
        girotest_pos.y = pos.y + 0.5f;
        girotest_pos.z = pos.z;
        
        GameObject.Find("Cube_girotest").GetComponent<Transform>().position = girotest_pos;
       

        //Получаем параметры GPS для автопилота

        Vector3 target;
        Vector3 gps_XHZ;
        GetGPS(out gps_XHZ);
        /*
        kalman_gps_x.Correct(gps_XHZ.x);
        geo_X = kalman_gps_x.State;
        kalman_gps_z.Correct(gps_XHZ.z);
        geo_Z = kalman_gps_z.State;
        */
        double geo_K = 0.01;
        geo_X = geo_X + (gps_XHZ.x - geo_X) * geo_K;
        geo_Z = geo_Z + (gps_XHZ.z - geo_Z) * geo_K;
        


        GetH(out H_);
        //H_ = gps_XHZ.y;

        //Вычисляем необходимые параметры крена, тангажа и газа, зля достижения заданных координат
        double[] dthr = new double[1];
        dthr[0] = H_PID.calc(H_, targetH);
        dthr[0] = dthr[0] < 0 ? 0 : dthr[0];
        throttle = dthr[0] = dthr[0] > 100 ? 100 : dthr[0];

        //Save data
      /*  
        System.IO.FileInfo file = new System.IO.FileInfo("data_time_h_thr.txt");
        String text;
        if (file.Exists == false) //Если файл не существует
        {
            file.Create(); //Создаем
            text = "Time; " + "H; " + "TargH;" + "Ay;" + "thrott";
        }
        else
        {
            text = Time.time.ToString() + "; " + H_.ToString() + "; " + targetH.ToString() + "; " + Ay.ToString() + "; " + throttle.ToString();
        }
        System.IO.StreamWriter stream_writer;
        stream_writer = file.AppendText();
        stream_writer.WriteLine(text);
        stream_writer.Close();
      */
        
        
         if (H_neyro_on)
         {
             m_Net_H.NetOUT(deltaH, out dthr);
             dthr[0] *= 100.0;
             dthr[0] = dthr[0] < 0 ? 0 : dthr[0];
             thrLimit = dthr[0] > 100 ? 100 : dthr[0];
             throttle = thrLimit;
         }
         else
         {
             dthr[0] /= 100.0;
            // do {
                 Kerr = m_Net_H.LernNW(deltaH, dthr, Klern);
            // } while (Kerr > 1E-4);

             m_Net_H.NetOUT(deltaH, out dthr);
             dthr[0] *= 100.0;
             dthr[0] = dthr[0] < 0 ? 0 : dthr[0];
             thrLimit = dthr[0] > 100 ? 100 : dthr[0];
            throttle = thrLimit;
        }
         
        target.y = 0;
        double LimitAngle = 40;

        double dPitch = Z_PID.calc(geo_Z, targetZ); 
        dPitch = dPitch < -LimitAngle ? -LimitAngle : dPitch;
        dPitch = dPitch > LimitAngle ? LimitAngle : dPitch;
        target.z = (float)dPitch;

        double dRoll = X_PID.calc(geo_X, targetX);
        dRoll = dRoll < -LimitAngle ? -LimitAngle : dRoll;
        dRoll = dRoll > LimitAngle ? LimitAngle : dRoll;
        target.x = (float)dRoll;

//neyro gps
        
         
        GameObject.Find("Acube").GetComponent<Transform>().position = girotest_pos;
        GameObject.Find("Acube").GetComponent<Transform>().rotation = Quaternion.FromToRotation(GameObject.Find("Acube").GetComponent<Transform>().rotation.eulerAngles, Vector3.zero);
        //GameObject.Find("Acube").GetComponent<Transform>().Rotate(accel_rot);

       

        double[] m_Y_roll = new double[1];
        m_Y_roll[0] = (target.x + 40.0) / 80.0;

        double[] m_Y_pich = new double[1];
        m_Y_pich[0] = (target.z + 40.0) / 80.0;

        Vector3 n_target;
        if (neyro_gps_on)
        {
            m_Net_roll.NetOUT(m_X_roll, out m_Y_roll);
            net_roll = (m_Y_roll[0]*80.0) -40.0f;
            target.x = (float)net_roll;

            m_Net_pich.NetOUT(m_X_pich, out m_Y_pich);
            net_pich = (m_Y_pich[0] * 80.0) - 40.0f;
            target.z = (float)net_pich;
        }
        else
        {
           // do {
                Kerr = m_Net_roll.LernNW(m_X_roll, m_Y_roll, Klern);
          //  } while (Kerr > 1E-2);
            m_Net_roll.NetOUT(m_X_roll, out m_Y_roll);
            net_roll = m_Y_roll[0]*80.0 - 40.0f;
            target.x = (float)net_roll;

            // do {
            Kerr = m_Net_pich.LernNW(m_X_pich, m_Y_pich, Klern);
            //  } while (Kerr > 1E-2);
            m_Net_pich.NetOUT(m_X_pich, out m_Y_pich);
            net_pich = m_Y_pich[0] * 80.0 - 40.0f;
            target.z = (float)net_pich;

            NetSave();
        }
  //neyroend
        //поворачиваем расчетные значения на отклонение от севера
        target = (Quaternion.Euler(target.x, 0, target.z)* Quaternion.Euler(0, rot.y, 0)).eulerAngles;
        n_target.x = (float)net_roll;
        n_target.z = (float)net_pich;
        n_target = (Quaternion.Euler(n_target.x, 0, n_target.z) * Quaternion.Euler(0, rot.y, 0)).eulerAngles;
        GameObject.Find("Acube").GetComponent<Transform>().Rotate(n_target);

        // targetYaw = (float)(360.0f / 2 * Math.PI) * (float)Math.Asin((targetX - geo_X) / Math.Sqrt(Math.Pow((targetX - geo_X), 2) + Math.Pow((targetZ - geo_Z), 2))); 
        //применяем расчитанные допустимые целевые значения крена и тангажа

        targetPitch = target.z;
        targetRoll = -target.x;

        //углы расчитанные из акселерометров

        pitch = accel_rot.x;
        yaw = rot.y; //rot.y = accel_rot.y;  направление севера нужно определять по компасу, аксели тут не помогут
        roll = accel_rot.z;
        
        /*
        //реальные углы
        pitch = rot.x;
        yaw = rot.y;
        roll = rot.z;
        */

        String pitch_str = "pitch: " + pitch.ToString();

    }

	//функция стабилизации квадрокоптера
	//с помощью PID регуляторов мы настраиваем
	//мощность наших моторов так, чтобы углы приняли нужные нам значения
	void stabilize () {

		//нам необходимо посчитать разность между требуемым углом и текущим
		//эта разность должна лежать в промежутке [-180, 180] чтобы обеспечить
		//правильную работу PID регуляторов, так как нет смысла поворачивать на 350
		//градусов, когда можно повернуть на -10

		double dPitch = targetPitch - pitch;
		double dRoll = targetRoll - roll;
		double dYaw = targetYaw - yaw;

		dPitch -= Math.Ceiling (Math.Floor (dPitch / 180.0) / 2.0) * 360.0;
		dRoll -= Math.Ceiling (Math.Floor (dRoll / 180.0) / 2.0) * 360.0;
		dYaw -= Math.Ceiling (Math.Floor (dYaw / 180.0) / 2.0) * 360.0;

        //1 и 2 мотор впереди
        //3 и 4 моторы сзади
        double motor1power = throttle;
        double motor2power = throttle;
        double motor3power = throttle;
        double motor4power = throttle;

        //ограничитель на мощность подаваемую на моторы,
        //чтобы в сумме мощность всех моторов оставалась
        //одинаковой при регулировке
        double powerLimit = throttle > 20 ? 20 : throttle;

		//управление тангажем:
		//на передние двигатели подаем возмущение от регулятора
		//на задние противоположное возмущение
		double pitchForce = - pitchPID.calc (0, dPitch / 180.0);
		pitchForce = pitchForce > powerLimit ? powerLimit : pitchForce;
		pitchForce = pitchForce < -powerLimit ? -powerLimit : pitchForce;
		motor1power +=   pitchForce;
		motor2power +=   pitchForce;
		motor3power += - pitchForce;
		motor4power += - pitchForce;

		//управление креном:
		//действуем по аналогии с тангажем, только регулируем боковые двигатели
		double rollForce = - rollPID.calc (0, dRoll / 180.0);
		rollForce = rollForce > powerLimit ? powerLimit : rollForce;
		rollForce = rollForce < -powerLimit ? -powerLimit : rollForce;
		motor1power +=   rollForce;
		motor2power += - rollForce;
		motor3power += - rollForce;
		motor4power +=   rollForce;

		//управление рысканием:
		double yawForce = yawPID.calc (0, dYaw / 180.0);
		yawForce = yawForce > powerLimit ? powerLimit : yawForce;
		yawForce = yawForce < -powerLimit ? -powerLimit : yawForce;
		motor1power +=   yawForce;
		motor2power += - yawForce;
		motor3power +=   yawForce;
		motor4power += - yawForce;

        kalman_t1.Correct(motor1power);
        motor1power = kalman_t1.State;
        GameObject.Find ("Motor1").GetComponent<motorScript>().power = motor1power;

        kalman_t2.Correct(motor2power);
        motor2power = kalman_t2.State;
        GameObject.Find ("Motor2").GetComponent<motorScript>().power = motor2power;

        kalman_t3.Correct(motor3power);
        motor3power = kalman_t3.State;
        GameObject.Find ("Motor3").GetComponent<motorScript>().power = motor3power;

        kalman_t4.Correct(motor4power);
        motor4power = kalman_t4.State;
        GameObject.Find ("Motor4").GetComponent<motorScript>().power = motor4power;
	}
    void OnGUI()
    {
        /*
        Vector2 gPos = new Vector2(10, 10);
        GUI.BeginGroup(new Rect(10, 10, 100, 100));
        Vector2 convertedGUIPos = GUIUtility.GUIToScreenPoint(gPos);
        GUI.EndGroup();
        Debug.Log("GUI: " + gPos + " Screen: " + convertedGUIPos);
        */
        //Выводим показания акселерометра
        String Ax_string = "Ax = " + Ax.ToString();
        String Ay_string = "Ay = " + Ay.ToString();
        String Az_string = "Az = " + Az.ToString();
        GUI.contentColor = Color.green; // 0x81EE8FFF;
        GUI.Label(new Rect(30, 20, 2000, 30), Ax_string);
        GUI.Label(new Rect(30, 40, 2000, 30), Ay_string);
        GUI.Label(new Rect(30, 60, 2000, 30), Az_string);

        //Выводим показания GPS 
        String H_string = "H = " + H_.ToString();
        String geo_X_string = "geo_X = " + geo_X.ToString();
        String geo_Z_string = "heo_Z = " + geo_Z.ToString();

        String targetH_str = "tH = " + targetH.ToString();
        String targetX_str = "tX = " + targetX.ToString();
        String targetZ_str = "tZ = " + targetZ.ToString();

        GUI.Label(new Rect(30, 90, 2000, 30), H_string);
        GUI.Label(new Rect(30, 110, 2000, 30), targetH_str);

        GUI.Label(new Rect(30, 140, 2000, 30), geo_X_string);
        GUI.Label(new Rect(30, 160, 2000, 30), targetX_str);

        GUI.Label(new Rect(30, 190, 2000, 30), geo_Z_string);
        GUI.Label(new Rect(30, 210, 2000, 30), targetZ_str);
       
        //Выводим время
        String time_str = Time.time.ToString();
        GUI.Label(new Rect(30, 250, 2000, 30), time_str);
    }

    //как советуют в доке по Unity вычисления проводим в FixedUpdate, а не в Update
    void FixedUpdate () {
        readControl();
        readRotation ();
		stabilize ();
        OnGUI();

    }
	
}

public class PID {
	
	private double P;
	private double I;
	private double D;
	
	private double prevErr;
	private double sumErr;
	
	public PID (double P, double I, double D) {
		this.P = P;
		this.I = I;
		this.D = D;
	}
	
	public double calc (double current, double target) {
		
		double dt = Time.fixedDeltaTime;
		
		double err = target - current;
		this.sumErr += err;
		
		double force = this.P * err + this.I * this.sumErr * dt + this.D * (err - this.prevErr) / dt;
		
		this.prevErr = err;
		return force;
	}
	
};
