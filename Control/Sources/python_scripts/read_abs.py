import socket
import json
import math

ABS_IPS = ['10.10.10.100', '10.10.10.101', '10.10.10.102', 
           '10.10.10.60', '10.10.10.61', '10.10.10.62', '10.10.10.63', '10.10.10.64', '10.10.10.65', 
           '10.10.10.80', '10.10.10.81', '10.10.10.82', '10.10.10.83', '10.10.10.84', '10.10.10.85',
            '10.10.10.20', '10.10.10.21', '10.10.10.22', '10.10.10.23', '10.10.10.24', '10.10.10.25', '10.10.10.26',
            '10.10.10.40', '10.10.10.41', '10.10.10.42', '10.10.10.43', '10.10.10.44', '10.10.10.45', '10.10.10.46']

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.settimeout(0.1)
remote_port_number = 2334

abs_angle_dict = {}
motor_rotor_angle_dict = {}

# Get the abs angle of each device
def get_abs_angle(address_list,angle_dict):
    data_dict = {
        "id": 1,
        "method": "Encoder.Angle",
        "params": "",
    }
    json_string = json.dumps(data_dict)
    
    for i in address_list:
        try:
            tcp_socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_socket_client.settimeout(0.1)
            tcp_socket_client.connect((i,remote_port_number))
            tcp_socket_client.send(json_string.encode('utf-8'))
            received_data = tcp_socket_client.recv(1024)

            json_object = json.loads(received_data.decode('utf-8'))
            if json_object['id'] == 1:
                abs_data = {"angle":json_object['result']['angle'],"radian":json_object['result']['radian'] }
                angle_dict[i] = abs_data
            tcp_socket_client.close()
        except OSError as e:
            print(f"{i}: get abs angle failed! {e}")

    
def get_motor_rotor_abs_pos(address_list,motor_rotor_angle_dict):
    data_dict = {
        "method": "GET",
        "reqTarget": "/abs_encoder",
    }
    json_string = json.dumps(data_dict)
    for i in address_list:
        try:
            udp_socket.sendto(str.encode(json_string), (i, remote_port_number))
            received_data, address = udp_socket.recvfrom(1024)
            json_object = json.loads(received_data.decode('utf-8'))
        except:
            json_object = {"status":"OK","reqTarget":"/abs_encoder","abs_pos":0}

        if json_object['status'] == "OK":
            abs_data = {"motor_rotor_abs_pos": int(json_object['abs_pos']) * 2 * math.pi / 16384}
            motor_rotor_angle_dict[i] = abs_data
            # print(f"ip: {i} json_object: {json_object}")
        else:
            print(f"ip {i} get motor rotor abs pos failed!")


def read_motor_rotor_abs_pos():
    ips = ['10.10.10.90', '10.10.10.91', '10.10.10.92', 
    '10.10.10.50', '10.10.10.51', '10.10.10.52', '10.10.10.53', '10.10.10.54', '10.10.10.55', 
    '10.10.10.70', '10.10.10.71', '10.10.10.72', '10.10.10.73', '10.10.10.74', '10.10.10.75',
    '10.10.10.10', '10.10.10.11', '10.10.10.12', '10.10.10.13', '10.10.10.14', '10.10.10.15', '10.10.10.16',
    '10.10.10.30', '10.10.10.31', '10.10.10.32', '10.10.10.33', '10.10.10.34', '10.10.10.35', '10.10.10.36']
    motor_enc_dict = {}
    get_motor_rotor_abs_pos(ips, motor_enc_dict)
    i = 0
    for key, value in motor_enc_dict.items():
        motor_rotor_angle_dict[ABS_IPS[i]] = value
        i += 1
        

def merge_dicts(dict1, dict2):
    merged_dict = dict1.copy()
    for key, value in dict2.items():
        if key in merged_dict:
            merged_dict[key].update(value)
        else:
            merged_dict[key] = value
    return merged_dict


def main():
    abs_file = open("abs.json",mode="w+",encoding="utf-8")
    get_abs_angle(ABS_IPS, abs_angle_dict)
    print("read abs complete!")
    read_motor_rotor_abs_pos()
    print("read motor rotor abs complete!")
    all_angle_dict = merge_dicts(abs_angle_dict, motor_rotor_angle_dict)
    
    json_string = json.dumps(all_angle_dict,indent=4)
    abs_file.write(json_string)
    

if __name__ == '__main__':
    main()
