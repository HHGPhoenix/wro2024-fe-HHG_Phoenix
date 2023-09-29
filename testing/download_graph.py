import paramiko
from scp import SCPClient

def copy_file_to_windows(ip, username, password, remote_file_path, local_file_path):
    try:
        # Create an SSH client
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # Connect to the Raspberry Pi
        ssh_client.connect(ip, username=username, password=password)

        # Create an SCP client
        with SCPClient(ssh_client.get_transport()) as scp:
            # Copy the remote file to the local system
            scp.get(remote_file_path, local_file_path)

        print(f"File copied successfully to {local_file_path}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the SSH connection
        ssh_client.close()

if __name__ == "__main__":
    # Input your Raspberry Pi's IP address, username, password, and file paths
    raspberrypi_ip = "pi.local"
    username = "pi"
    password = "pi"
    remote_file_path = r"/home/pi/wro/testing/HoldLine.txt"
    local_file_path = r"C:\Users\Felix\OneDrive - Helmholtz-Gymnasium\Desktop\Github\wro2024-fe-HHG_Phoenix\wro2024-fe-HHG_Phoenix\testing\HoldLine.txt"

    # Call the function to copy the file
    copy_file_to_windows(raspberrypi_ip, username, password, remote_file_path, local_file_path)
