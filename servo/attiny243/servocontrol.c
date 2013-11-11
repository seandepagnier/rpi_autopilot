void spi_init()
{
    /* setup as SPI slave */
    DDR_SPI |= _BV(DD_MISO); /* slave out as master */
    SPCR |= _BV(SPE); /* enable spi */
}

uint8_t spi_transfer(uint8_t byte)
{
    SPDR = byte;
    while(!(SPSR & (1<<SPIF)));
    return SPDR;
}

void pwm_init()
{
    /* set pins to output */
    /* setup timer interrupt */
    TCCR1A = 0; // normal counting
    TCCR1B = _BV(CS11); // prescaler of 8
    TCNT1 = 0; // clear count
    TIFR1 = _BV(OCRF1A); // clear interrupts
    TIMSK1 = _BV(OCIE1A); // enable output compare interrupt
}

void pwm_set(uint8_t channel, int16_t command)
{

}

void adc_init()
{
}

int16_t adc_read(uint8_t current)
{
}

#define RS(ST) do { if(ST !='n') goto reswitch; } while (0)

int main()
{
    spi_init();
    pwm_init();
    adc_init();

    uint16_t current, voltage;

    for(;;) {
        char value = spi_transfer('k');
    reswitch:
        switch(value) {
        case 'c': /* capabilities */
            RS(spi_transfer('2')); /* 2 servos */
            RS(spi_transfer('m')); /* voltage/current meters */
            RS(spi_transfer('d')); /* done */
            break;
        case 'm': /* read meters */
            RS(spi_transfer(voltage));
            RS(spi_transfer(current));
            break;
        case 'k' /* okay? used to syncronize */
            break;
        case '1': /* command channel */
        case '2':
        {
            uint8_t hb, lb;
            RS((hb = spi_transfer('n')));
            RS((lb = spi_transfer('n')));

            int16_t cmd = ((uint16_t)hb << 8) | lb;
            pwm_set(value - '1', cmd);
        } break;
        }
        /* read current and voltage */
        voltage = read_adc(0);
    }
}
