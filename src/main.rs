use esp_idf_svc::hal::{
    adc::{
        attenuation::DB_12,
        oneshot::{config::AdcChannelConfig, config::Calibration, AdcChannelDriver, AdcDriver},
    },
    delay::{Ets, FreeRtos},
    peripherals::Peripherals,
};

/// Duración de la ventana de muestreo en microsegundos.
/// 200 ms = 200_000 µs contiene exactamente:
/// - 12 ciclos completos a 60 Hz (12 * 16.666 ms = 200 ms)
const WINDOW_DURATION_US: i64 = 200_000;

/// Período entre muestras en microsegundos (~200 µs => ~5 kHz de frecuencia de muestreo,
/// es decir ~83 muestras por ciclo a 60 Hz).
const SAMPLE_PERIOD_US: u32 = 200;

/// Factor de calibración del módulo ZMPT101B: V_linea_RMS / V_adc_RMS.
/// Ajustado por regla de tres: 598.03 * (123.0 V / 107.0 V) = 687.46
const CALIBRATION_FACTOR: f64 = 687.46;

fn main() -> Result<(), esp_idf_svc::sys::EspError> {
    // Necesario para enlazar los parches de runtime de esp-idf-sys
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    // En el ESP32-S3, GPIO10 corresponde a ADC1_CH9
    let adc = AdcDriver::new(peripherals.adc1)?;

    // Configuramos atenuación de 12 dB (rango completo de ~0 a ~3100 mV en ESP32-S3)
    // y calibración por curva (Curve Fitting) en milivoltios
    let adc_config = AdcChannelConfig {
        attenuation: DB_12,
        calibration: Calibration::Curve,
        ..Default::default()
    };

    let mut zmpt_pin = AdcChannelDriver::new(&adc, peripherals.pins.gpio10, &adc_config)?;

    log::info!("Iniciando lectura True RMS del módulo ZMPT101B en GPIO10...");

    let mut samples: Vec<i64> = vec![0_i64; 1000];

    loop {
        let start_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
        // 1. Adquisición de muestras durante la ventana de 200 ms
        let mut n: usize = 0;
        while unsafe { esp_idf_svc::sys::esp_timer_get_time() } - start_us < WINDOW_DURATION_US
            && n < samples.len()
        {
            samples[n] = adc.read(&mut zmpt_pin)? as i64;
            n += 1;
            Ets::delay_us(SAMPLE_PERIOD_US);
        }
        // 2. Calcular el offset DC promedio
        let mut dc_offset_mv: i64 = 0;
        for j in 0..n {
            let val = samples[j];
            dc_offset_mv += val;
        }
        dc_offset_mv /= n as i64;
        // 3. Calcular True RMS de la componente AC (restando el offset DC de cada muestra)
        let mut sum_sq_ac: i64 = 0;
        for j in 0..n {
            let v_ac = samples[j] - dc_offset_mv; // Puede ser negativo, por eso usamos i64
            sum_sq_ac += v_ac * v_ac;
        }
        let adc_rms_mv: f64 = (sum_sq_ac as f64 / n as f64).sqrt();
        let ac_voltage_rms = adc_rms_mv * CALIBRATION_FACTOR / 1000.0;
        log::info!(
            "N: {:3} | DC: {:4} mV | ADC RMS: {:6.2} mV | True RMS: {:5.1} V AC",
            n,
            dc_offset_mv,
            adc_rms_mv,
            ac_voltage_rms
        );
        FreeRtos::delay_ms(500);
    }
}
