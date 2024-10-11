import json

abs_ip_addresses = ['10.10.10.100', '10.10.10.101', '10.10.10.102', 
                    '10.10.10.60', '10.10.10.61', '10.10.10.62', '10.10.10.63', '10.10.10.64', '10.10.10.65', 
                    '10.10.10.80', '10.10.10.81', '10.10.10.82', '10.10.10.83', '10.10.10.84', '10.10.10.85',
                    '10.10.10.20', '10.10.10.21', '10.10.10.22', '10.10.10.23', '10.10.10.24', '10.10.10.25', '10.10.10.26',
                    '10.10.10.40', '10.10.10.41', '10.10.10.42', '10.10.10.43', '10.10.10.44', '10.10.10.45', '10.10.10.46']

joint_names = [
    'waistRoll',  'waistPitch',  'waistYaw',
    'hipPitch_Right', 'hipRoll_Right', 'hipYaw_Right',  'kneePitch_Right', 'anklePitch_Right', 'ankleRoll_Right',
    'hipPitch_Left', 'hipRoll_Left', 'hipYaw_Left',  'kneePitch_Left', 'anklePitch_Left', 'ankleRoll_Left',
    'shoulderPitch_Left', 'shoulderRoll_Left', 'shoulderYaw_Left',  'elbow_Left', 'wristYaw_Left', 'wristPitch_Left', 'wristRoll_Left',
    'shoulderPitch_Right', 'shoulderRoll_Right', 'shoulderYaw_Right',  'elbow_Right', 'wristYaw_Right', 'wristPitch_Right', 'wristRoll_Right',
]
def validate_abs():
    # Read abs.json
    # Check if abs is empty
    with open("abs.json", 'r', encoding='utf-8') as abs_angle_file:
        abs_angle_data = json.load(abs_angle_file)
    # Check if the number of abs is correct
    for ip in abs_ip_addresses:
        ip_exists = ip in abs_angle_data and 'motor_rotor_abs_pos' in abs_angle_data[ip] and 'radian' in abs_angle_data[ip]
        if not ip_exists:
            is_valid = False
            print(ip, " no abs")
    print("Check abs complete !")

def main():
    validate_abs()
    print(True)

if __name__ == '__main__':
    main()

