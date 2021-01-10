// Stel voedingspanning zo in, dat je op 10000 tr/min uitkomt bij PWM van 255!
// Communicatie via virtuele seriële poort op 9600 baud.
//
// Voorbeeld van te sturen JSON
// Sturen
// ------
// {"PWM":50}
// Regelen
// -------
// {"setpoint":2500,"Kp":2,"Ki":5,"Kd":0.1}
// setpoint = gewenste toeren/minuut
// Kp, Ki en Kd = regelparameters
// PWM = stuurwaarde (niet regelen dus). Tussen 0 en 255.
// rpm = actuele toeren/minuut
//
// De code verzendt zelf iedere 250ms een JSON-pakket met volgende opbouw:
//
// {"setpoint":2500,"PWM":43,"Kp":2.00,"Ki":5.00,"Kd":0.10,"rpm":2499,"ticks":254680}
//
// OPM: Reception timeout werd gerealiseerd met ticks...
//
// Project opbouwd voor de Keil µVision 5 IDE, voor STM32F091RC.
#include "stm32f091xc.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
void SystemClock_Config(void);
void InitIo(void);
void InitUart2(void);
void StringToUsart(char *string);
void SplitAndSaveJsonData(char *jsonString);
void InitTimer7(void);
void ResetTimer7(void);
void StartTimer7(void);
void StopTimer7(void);
uint16_t ReadTimer7(void);
void JsonToUsart(void);
void InitPwm(void);
void SetPwm(uint8_t pPwm);
#define YES 1
#define NO 0
#define LOOP_TIME 100 // Iedere 100 ms control loop laten uitvoeren.
enum usartState
{
    idle,
    busyReceiving,
    newStringArrived,
    receptionTimeOut,
    overflowOccured
};
volatile enum usartState usart2State = idle;
- 1 -
    uint8_t usart2ReceiverPointer = 0; // Pointer die bijhoudt waar in het array je mag schrijven.
char usart2ReceivedData[200];          // Maximum 200 karakters data aanvaarden.
char tekst[100];
char jsonData[30][40]; //JSON data in rijen (30) en kolommen (40)
volatile uint8_t usartReceiverPointer = 0, usartReceiverTimeOut = 0;
uint32_t temp = 0;
uint8_t enableControlLoop = 0; // Is er via JSON PWM-info binnengekomen?
volatile uint32_t pwmTicks = 0, ticks = 0, uartTimeOutTicks = 0;
volatile uint8_t pwm = 0;
float scale = 0;                              // Schaalwaarde om over te gaan van PWM naar tr/min.
float error = 0;                              // Errorsignaal, in tr/min.
float Kp = 0;                                 // Regelparameter voor de P-regeling.
float Ki = 0;                                 // Regelparameter voor de I-regeling.
float Kd = 0;                                 // Regelparameter voor de D-regelaar.
float errorValue = 0, errorValuePrevious = 0; // Error na herschalen.
float controlValue = 0;                       // De regelwaarde (vóór afronding naar PWM).
float proportionalValue = 0, integralValue = 0, derivativeValue = 0;
// Voor de eenvoud van omzetten van UART, alle varialbelen in float voorzien.
float pwmFloat = 0;
float setpoint = 0;
volatile uint32_t timerValueRpmTemp = 0, timerValueRpm = 0; // Timer waarde voor het opmeten van de omwentelingstijd van
de motor.
    // De tijd wordt opgeslaan in microseconden...
    volatile uint32_t rpm = 0; // Toerental van de motor tr/min.
int main(void)
{
    SystemClock_Config();
    // Initialisaties
    InitIo();
    InitUart2();
    InitTimer7();
    InitPwm();
    // Oneindige lus starten.
    while (1)
    {
        // Is er nieuwe data vanuit de virtuele seriële poort ontvangen (JSON)?
        // Splits die, sla ze op en toon ze via de USART.
        if (usart2State == newStringArrived)
        {
            SplitAndSaveJsonData(usart2ReceivedData);
            JsonToUsart();
            usart2State = idle;
        }
        // Er was een time out op de virtuele seriële poort.
        if (usart2State == receptionTimeOut)
        {
            // Byte array opbouwen
            -2 -
                strcpy(tekst, "\r\ntime out\r\n\r\n");
            StringToUsart(tekst);
            usart2State = idle;
        }
        // Meer dan 200 karakters verstuurd op de poort, overflow.
        if (usart2State == overflowOccured)
        {
            // Byte array opbouwen
            strcpy(tekst, "\r\noverflow\r\n\r\n");
            StringToUsart(tekst);
            usart2State = idle;
        }
        // Automatisch alle info versturen in JSON-formaat iedere 250ms.
        if (pwmTicks % 250 == 0)
            JsonToUsart();
    }
}
void InitPwm(void)
{
    ...
}
void SetPwm(uint8_t pPwm)
{
    ...
}
void JsonToUsart(void)
{
    sprintf(tekst, "{\"setpoint\":%.0f,\"PWM\":%d,\"Kp\":%.2f,\"Ki\":%.2f,\"Kd\":%.2f,\"rpm\":%u,\"ticks\":%u}\r\n", setpoint,
            pwm, Kp, Ki, Kd, rpm, ticks);
    StringToUsart(tekst);
}
void StringToUsart(char *string)
{
    ...
}
void InitIo(void)
{
    ...
}
void EXTI4_15_IRQHandler(void)
{
    // Als het een interrupt is van PA10 (HALL/IR-sensor), toggle de LED.
    if ((EXTI->PR & EXTI_PR_PR10) == EXTI_PR_PR10)
        -3 -
        {
            // Interrupt (pending) vlag wissen
            // In interrupt subroutine, de 'pending register' vlag resetten door
            // een 1 te schrijven: EXTI->PR |= EXTI_PR_PR13;
            EXTI->PR |= EXTI_PR_PR10;
            // LED1 toggle'n als bewijs van interrupt.
            GPIOC->ODR = GPIOC->ODR ^ GPIO_ODR_0;
            // LED2 uitschakelen (indicatie te trage omwentelingssnelheid).
            GPIOB->ODR = GPIOB->ODR & ~GPIO_ODR_3;
            timerValueRpmTemp += ReadTimer7() / 2; // Timer7 waarde is uitgedrukt in 0,5µs. Dus deel deze
            // waarde door twee om het resultaat in microseconden te bekomen.
            timerValueRpm = timerValueRpmTemp; // De huidig opgemeten timerwaarde van Timer7 opslaan.
            // Later daarmee een toerental berekenen.
            timerValueRpmTemp = 0;
            StartTimer7(); // Volgende meting starten.
        }
}
void InitUart2(void)
{
    ...
}
// Interrupt handler van USART2
void USART2_IRQHandler(void)
{
    uint8_t temp = 0;
    // Bytes ontvangen?
    if ((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
    {
        // Byte ontvangen, lees hem om alle vlaggen te wissen.
        temp = USART2->RDR;
        if (temp == 123) // { = 123
        {
            // indien {, start
            usart2State = busyReceiving;
            // Pointer resetten
            usart2ReceiverPointer = 0;
            // Byte array opbouwen
            usart2ReceivedData[usart2ReceiverPointer++] = temp;
            // Start timer die UART-timeout in de gaten houdt.
            uartTimeOutTicks = ticks;
        }
        -4 -
            else
        {
            if (usart2State == busyReceiving)
            {
                usart2ReceivedData[usart2ReceiverPointer] = temp;
                // Indien '}' (125) gezien, geef signaal.
                if (usart2ReceivedData[usart2ReceiverPointer] == 125)
                {
                    StopTimer7();
                    usart2State = newStringArrived;
                }
                else
                {
                    if (usart2ReceiverPointer < 200) // geen overflow?
                        usart2ReceiverPointer++;
                    else
                        usart2State = overflowOccured;
                }
            }
        }
    }
}
// Functie de een JSON-string binnenkrijgt, die splits en opslaat in de juiste float variabelen.
void SplitAndSaveJsonData(char *jsonString)
{
    uint8_t jsonIndexer = 0;
    char *splitTextPointer;
    char tekst[50];
    // String splitten op {:,}".
    splitTextPointer = strtok(jsonString, "{:,}\"");
    // JSON-index resetten
    jsonIndexer = 0;
    // Zolang data aanwezig...
    while (splitTextPointer != NULL)
    {
        // Data opslaan in tijdelijke string.
        sprintf(tekst, "%s", splitTextPointer);
        // Data kopiëren naar JSON array
        strcpy(jsonData[jsonIndexer++], tekst);
        // Volgende split uitvoeren.
        splitTextPointer = strtok(NULL, "{:,}\"");
    }
    // Ervan uit gaan dat er geen PWM-info werd meegestuurd en dat er dus 'geregeld' moet worden.
    enableControlLoop = 1;
    -5 -
        for (temp = 0; temp < jsonIndexer; temp++)
    {
        if (strcmp(jsonData[temp], "setpoint") == 0)
            setpoint = atof(jsonData[temp + 1]);
        if (strcmp(jsonData[temp], "PWM") == 0)
        {
            enableControlLoop = 0; // Er is info meegegeven voor de PWM,
            // dus zorg ervoor dat er gestuurd wordt en niet geregeld.
            pwmFloat = atof(jsonData[temp + 1]);
        }
        if (strcmp(jsonData[temp], "Kp") == 0)
            Kp = atof(jsonData[temp + 1]);
        if (strcmp(jsonData[temp], "Ki") == 0)
            Ki = atof(jsonData[temp + 1]);
        if (strcmp(jsonData[temp], "Kd") == 0)
            Kd = atof(jsonData[temp + 1]);
        temp++; // Eén stap overslaan, omdat JSON-data per paren zit.
    }
    // Als niet geregeld, maar gestuurd wordt. De regelparameters op nul zetten. Zodat voor de gebruiker zeker
    // duidelijk wordt dat er niet geregeld wordt.
    if (enableControlLoop == 0)
    {
        setpoint = 0;
        Kp = 0;
        Ki = 0;
        Kd = 0;
        integralValue = 0;
    }
}
// Handler die iedere 1ms afloopt. Ingesteld met SystemCoreClockUpdate() en SysTick_Config().
void SysTick_Handler(void)
{
    // Globale timer bijhouden (1 tick per milliseconde).
    ticks++;
    // Controleren of er een Time-Out is geweest bij het ontvangen van de UART-data.
    // Time-out van 1 seconde ...
    if ((usart2State == busyReceiving) && (ticks > uartTimeOutTicks + 1000))
        usart2State = receptionTimeOut;
    // Regellus maken
    if (ticks % LOOP_TIME == 0) // Iedere 100 millisecondenseconden de regelaar laten 'regelen.
    {
        scale = 255.0 / 10000.0; // Maximum PWM is 255. Als je die aanhoudt, klimt het
        -6 -
            // toerental tot 10000 tr/min (voeding zo instellen).
            // Indien de opgemeten tijd van één toer via Timer7 geldig is, ...
            if (timerValueRpm > 0)
        {
            // 1 seconde duurt 1000000 stappen van Timer 7 (want timerValueRpm is uitgedrukt in microseconden).
            // Stel: 250 000 stappen = 250 000 / 1 000 000 seconden = 250 ms per toer
            // Toerental is dan = 60/0.25 = 240 tr/min
            // of 60 * 1 000 000 / 250 000 = 240
            rpm = (60 * 1000000) / timerValueRpm;
        }
        else rpm = 0;
        // Fout berekenen (in toeren per minuut).
        error = setpoint - (float)rpm;
        // De error-waarder herschalen om er een stuurwaarde van te maken.
        errorValue = error * scale;
        // P-regelaar
        proportionalValue = errorValue * Kp;
        // I-regelaar
        // Wind-up tegen gaan.
        if (!(((integralValue <= -2550) && (errorValue < 0)) || ((integralValue >= 2550) && (errorValue > 0))))
            // LOOP_TIME delen door 1000 omdat dit een waarde in milliseconden is.
            integralValue = integralValue + Ki * (errorValue * (((float)LOOP_TIME) / 1000.0));
        if (Ki == 0)
            integralValue = 0;
        // D-regelaar
        derivativeValue = (errorValue - errorValuePrevious) / (((float)LOOP_TIME) / 1000.0);
        derivativeValue = derivativeValue * Kd;
        errorValuePrevious = errorValue;
        // Stuursignaal berekenen uit P, I en D.
        controlValue = proportionalValue + integralValue + derivativeValue;
        // Stuurwaarde begrenzen tot 255, want dat is een PWM-signaal geworden...
        if (controlValue > 255)
            controlValue = 255;
        if (controlValue < 0)
            controlValue = 0;
        // Keuze maken tussen sturen vanuit de PC of de regellus haar werk laten doen.
        if (enableControlLoop == 0)
            pwm = (uint8_t)pwmFloat; // Sturen
        else
        {
            -7 -
                pwm = (uint8_t)controlValue; // Regelen (en stuurwaarde resetten voor de duidelijkheid).
            pwmFloat = 0;
        }
        // PWM updaten naar de PWM-generator (gemaakt met Timer1).
        SetPwm(pwm);
    }
}
// Timer 7 instellen (voor bepalen toerental).
void InitTimer7(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // clock voorzien voor de timer
    TIM7->PSC = 24;                     // 48 000 000 / 24 = 2 000 000 Hz of per stap 0,5µs.
    TIM7->CNT = 0;                      // Teller op nul zetten.
    TIM7->ARR = 50000;                  // Als je iedere 25 milliseconden (25 000 µs) een interrupt wil...
    // OPM: ARR op nul zetten, zorgt voor het disable'n van de timer.
    TIM7->DIER |= TIM_DIER_UIE;     // Interrupt enable voor timer 6
    TIM7->CR1 &= ~TIM_CR1_CEN;      // counter disable
    NVIC_SetPriority(TIM7_IRQn, 3); // Kies een prioriteit...
    NVIC_EnableIRQ(TIM7_IRQn);      // Enable interrupt
}
// Interrupt van Timer 7 opvangen.
// Per 25 000 µs een interrupt. Zorg ervoor dat dit cummuleert in timerValueRpm. Na 3 secondenan
// alles stop zetten....
void TIM7_IRQHandler(void)
{
    if ((TIM7->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        // Interruptvlag resetten
        TIM7->SR &= ~TIM_SR_UIF;
        // Er zijn 25ms gepasseerd, tel ze bij de timerValueRpm [microseconden].
        timerValueRpmTemp += 25000;
        // Gemeten tijd langer dan 3 seconden, stop ermee...
        if (timerValueRpmTemp >= 3000000)
        {
            // LED2 inschakelen als bewijs van te trage omwentelingssnelheid.
            GPIOB->ODR = GPIOB->ODR ^ GPIO_ODR_3;
            StopTimer7();
            ResetTimer7();
            timerValueRpm = 0; // Gemeten tijd op nul zetten. Later dan
            // gebruiken om toerental op nul uit te sturen.
            timerValueRpmTemp = 0;
        }
        -8 -
    }
}
void ResetTimer7(void){
    ...} uint16_t ReadTimer7(void)
{
    return TIM7->CNT;
}
void StartTimer7(void)
{
    ...
}
void StopTimer7(void)
{
    ...
}
void SystemClock_Config(void)
{
    ...
}