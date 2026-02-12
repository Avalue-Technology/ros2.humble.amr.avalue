
class PID {
public:
	float kp;
	float ki;
	float kd;
	float targetpoint;
	// ��һ�ε����
	//Last time error
	float prevError;
	// ����
	//integral
	float intergral;
	// ΢��
	//differential
	float derivative;

	PID(float kp, float ki, float kd);

	void Set_PID(float kp, float ki, float kd);

	/**
	 * pid calculation function pid�ļ��㺯��
	 * @param target  Ŀ��ֵ
	 * @param current ��ǰֵ
	 * @return  pwm
	 */
	float compute(float target, float current);

	/**
	 *  �������е����: �����õ��ٶ� �� ��һ�β�һ��
	 *  Reset all errors: When the set speed is different from the last time
	 */
	void reset();
};

