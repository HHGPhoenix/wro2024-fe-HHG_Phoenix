import requests
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def fetch_data(url):
    try:
        response = requests.get(url)
        if response.status_code == 200:
            return response.json()
        else:
            print("Failed to fetch data from the URL.")
            return None
    except Exception as e:
        print(f"Failed to fetch data from the URL. Exception: {e}")
        return None

def draw_graph():
    plt.cla()  # Clear the current axes.
    for key in data:
        y = data[key]
        x = list(range(len(y)))
        plt.plot(x, y, label=key)
    plt.legend(loc='upper left')
    plt.tight_layout()

# Replace 'url' with the actual URL from where you want to fetch the data
url = 'http://192.168.178.71:5000/data_feed'

data = {'angle': [], 'cpu_usage': [], 'ram_usage': [], 'voltage': []}

def animate(i): 
    new_data = fetch_data(url)
    if new_data:
        for key in data:
            if key in new_data:
                data[key].append(new_data[key])
                data[key] = data[key][-100:]  # Keep only the last 100 data points
    draw_graph()

ani = FuncAnimation(plt.gcf(), animate, interval=300)

plt.tight_layout()
plt.show()