use esp_idf_svc::hal::{
    adc::{
        attenuation::DB_12,
        oneshot::{config::AdcChannelConfig, config::Calibration, AdcChannelDriver, AdcDriver},
    },
    delay::FreeRtos,
    peripherals::Peripherals,
};

fn main() -> Result<(), esp_idf_svc::sys::EspError> {
    // Necesario para enlazar los parches de runtime de esp-idf-sys
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    // En el ESP32-S3, GPIO9 corresponde a ADC1_CH8
    let adc = AdcDriver::new(peripherals.adc1)?;

    // Configuramos atenuación de 12 dB (rango completo de ~0 a ~3100 mV en ESP32-S3)
    // y calibración por curva (Curve Fitting) en milivoltios
    let adc_config = AdcChannelConfig {
        attenuation: DB_12,
        calibration: Calibration::Curve,
        ..Default::default()
    };

    let mut adc_pin = AdcChannelDriver::new(&adc, peripherals.pins.gpio9, &adc_config)?;

    log::info!("Iniciando lectura ADC en GPIO9...");

    loop {
        let mv = adc.read(&mut adc_pin);

        println!("Valor leido = {:?} mv", mv);

        FreeRtos::delay_ms(1000);
    }
}
