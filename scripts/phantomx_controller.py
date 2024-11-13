#!/usr/bin/env python3

# Importación de librerías de ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Importación de librerías para la interfaz
import tkinter as tk
from tkinter import messagebox

class ROSPublisher(Node):
    def __init__(self):
        super().__init__("gui_publisher")
        self.publisher = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)

    def publish_values(self, values):
        msg = Float64MultiArray()
        msg.data = values
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {values}")

class GUIApp:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("Publicador de ROS2")
        
        self.entries = []
        self.setup_ui()
    
    def setup_ui(self):
        # Crear campos de entrada para cada valor
        for i in range(5):
            label = tk.Label(self.root, text=f"Valor {i + 1} (-3.14 a 3.14):")
            label.grid(row=i, column=0, padx=10, pady=5)

            entry = tk.Entry(self.root)
            entry.grid(row=i, column=1, padx=10, pady=5)
            self.entries.append(entry)

        # Crear botón para enviar
        send_button = tk.Button(self.root, text="Enviar", command=self.send_values)
        send_button.grid(row=5, column=0, columnspan=2, pady=10)

    def send_values(self):
        values = []
        try:
            for entry in self.entries:
                value = float(entry.get())
                if -3.14 <= value <= 3.14:
                    values.append(value)
                else:
                    raise ValueError("Valor fuera de rango")

            # Publicar los valores en el topic ROS2
            self.node.publish_values(values)
            messagebox.showinfo("Éxito", "Valores enviados correctamente.")

        except ValueError:
            messagebox.showerror("Error", "Ingrese valores numéricos entre -3.14 y 3.14 en todos los campos.")

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    ros_node = ROSPublisher()

    app = GUIApp(ros_node)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
