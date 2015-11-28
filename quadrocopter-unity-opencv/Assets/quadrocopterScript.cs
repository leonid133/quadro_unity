using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System;


public class quadrocopterScript : MonoBehaviour {

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
	//каждому углу свой регулятор, класс PID определен ниже
	
	private PID pitchPID = new PID (100, 0, 20);
	private PID rollPID = new PID (100, 0, 20);
	private PID yawPID = new PID (50, 0, 50);

	private Quaternion prevRotation = new Quaternion (0, 1, 0, 0);

    //mouse contol
    float x = 0.0f;
    float y = 0.0f;
    float throttleSpeed = 0.01f;
    float YawSpeed = 2.5f;
    float RollSpeed = 2.5f;
    float PitchSpeed = 2.5f;

    void readControl() {

        
        throttle += Input.GetAxis("Vertical")* throttleSpeed;
        targetYaw += Input.GetAxis("Horizontal")* YawSpeed;
        if (Input.GetMouseButton(0))
        {
            
            targetRoll += (Input.GetAxis("Mouse X")) * RollSpeed;
            targetPitch -= (Input.GetAxis("Mouse Y")) * PitchSpeed;
        }
        
        x = Input.GetAxis("Mouse X");
        y = Input.GetAxis("Mouse Y");
        if (Input.GetKey("left ctrl"))
        {
            //throttle = 0;
            targetRoll = 0;
            targetPitch = 0;
            targetYaw = 0;
        }
        if (Input.GetKey("1"))
        {
            throttle = 10;
        }
        if (Input.GetKey("2"))
        {
            throttle = 20;
        }
        if (Input.GetKey("3"))
        {
            throttle = 30;
        }
        if (Input.GetKey("4"))
        {
            throttle = 40;
        }
        if (Input.GetKey("5"))
        {
            throttle = 50;
        }
    }

    void readRotation () {
		
		//фактическая ориентация квадрокоптера,
		//в реальном квадрокоптере эти данные необходимо получать
		//из акселерометра-гироскопа-магнетометра
		Vector3 rot = GameObject.Find ("Frame").GetComponent<Transform> ().rotation.eulerAngles;
        

        System.Random pitch_rnd = new System.Random();
        double pitch_err = Convert.ToDouble(pitch_rnd.Next(-1000,1000)) * 0.001f;
        pitch = rot.x + pitch_err;
        
        System.Random yaw_rnd = new System.Random();
        double yaw_err = Convert.ToDouble(yaw_rnd.Next(-1000, 1000)) * 0.001f;
        yaw = rot.y + yaw_err;

        System.Random roll_rnd = new System.Random();
        double roll_err = Convert.ToDouble(roll_rnd.Next(-1000, 1000)) * 0.001f;
        roll = rot.z + roll_err;

        /*
        pitch = rot.x;
        yaw = rot.y;
        roll = rot.z;
        */
        String pitch_str = "pitch: " + pitch.ToString();

        //var text1 = GameObject.Find("Text1").GetComponent<GUIText>();
        //text1.text = "111";
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
        /*
        System.Random motor1_rnd = new System.Random();
        double motor1_err = Convert.ToDouble(motor1_rnd.Next(-1000, 1000)) * 0.0001f;
        System.Random motor2_rnd = new System.Random();
        double motor2_err = Convert.ToDouble(motor2_rnd.Next(-1000, 1000)) * 0.0001f;
        System.Random motor3_rnd = new System.Random();
        double motor3_err = Convert.ToDouble(motor3_rnd.Next(-1000, 1000)) * 0.0001f;
        System.Random motor4_rnd = new System.Random();
        double motor4_err = Convert.ToDouble(motor4_rnd.Next(-1000, 1000)) * 0.0001f;

        double motor1power = throttle + motor1_err;
		double motor2power = throttle + motor2_err;
		double motor3power = throttle + motor3_err;
		double motor4power = throttle + motor4_err;
        */
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

		GameObject.Find ("Motor1").GetComponent<motorScript>().power = motor1power;
		GameObject.Find ("Motor2").GetComponent<motorScript>().power = motor2power;
		GameObject.Find ("Motor3").GetComponent<motorScript>().power = motor3power;
		GameObject.Find ("Motor4").GetComponent<motorScript>().power = motor4power;
	}

	//как советуют в доке по Unity вычисления проводим в FixedUpdate, а не в Update
	void FixedUpdate () {
        readControl();
        readRotation ();
		stabilize ();
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
