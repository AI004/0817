import socket
import json

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.settimeout(1)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

network_address = '10.10.10.255'
remote_port_number = 2334
device_model = 'ABS_Encoder'
server_ip_list = ["10.10.10.224"]
abs_angle_dict = {}
json_dict = {}

# Broadcast to get device information, IP address, etc.
def broadcast_func():
    data_dict = {
        "id": 0,
        "method": "Device.Info",
        "params": "",
    }

    json_string = json.dumps(data_dict)
    found_abs = False
    ip_address_list = []

    for i in range(3): 
        udp_socket.sendto(json_string.encode('utf-8'), (network_address, remote_port_number)) 
        while True:
            try:
                received_data, address = udp_socket.recvfrom(1024) 
                device_json_object = json.loads(received_data.decode('utf-8'))
                if "result" in device_json_object:
                    if ip_address_list.count(address[0]) == 0 : 
                        if device_json_object['result']["dev_model"] == device_model:  
                            ip_address_list.append(address[0])  
                            json_dict[address[0]] = device_json_object['result']['serial_number']
                found_abs = True
            except socket.timeout:
                if found_abs:
                    break
                else:
                    break
    return ip_address_list

# Get the abs angle of each device
def get_angle(address_list,angle_dict):
    data_dict = {
        "id": 1,
        "method": "Encoder.Angle",
        "params": "",
    }
    json_string = json.dumps(data_dict)
    for i in address_list: 
        tcp_socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket_client.connect((i,remote_port_number))
        tcp_socket_client.send(json_string.encode('utf-8'))
        received_data = tcp_socket_client.recv(1024)
        
        json_object = json.loads(received_data.decode('utf-8'))  
        if json_object['id'] == 1:
            abs_data = {"angle":json_object['result']['angle'],"radian":json_object['result']['radian'] }
            angle_dict[i] = abs_data  
        tcp_socket_client.close();        

def main():
    abs_file = open("abs.json",mode="w+",encoding="utf-8")
    server_ip_list = broadcast_func()
    server_ip_list.sort()
    if len(server_ip_list) == 0:
        print('no abs in the network!')
        return
    get_angle(server_ip_list,abs_angle_dict)
    json_string = json.dumps(abs_angle_dict,indent=4)
    abs_file.write(json_string)
    print("read abs complete!") 

if __name__ == '__main__':
    main()

