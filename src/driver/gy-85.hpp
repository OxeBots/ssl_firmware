i2c_master_bus_handle_t bus_handle; 

GY85_I2C gy85;
gy85.init(bus_handle);

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

while (condition)
{
    /* condição para receber os dadosd do sensor */
}
 (true) {
    gy85.read_accel(ax, ay, az);
    gy85.read_gyro(gx, gy, gz);
    gy85.read_mag(mx, my, mz);

    ESP_LOGI("SENSORES", "ACC: %d %d %d | GYRO: %d %d %d | MAG: %d %d %d",
             ax, ay, az, gx, gy, gz, mx, my, mz);

    vTaskDelay(pdMS_TO_TICKS(500));
}
