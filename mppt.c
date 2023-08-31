// MPPT From ST Micro Inverter 250 W
void MPPT_func()
{
    CCR_Val = (u16) DCDC_GetDutyCycle();
    Vpanel=PV_Voltage;
    Ipanel=PV_Current;

    if(Ipanel==0) Ipanel=1;
    Pow = (u32)((u32)Vpanel * (u32)Ipanel);

    if(flag_mppt==1) { Step_modify(); }

    if((Pow >= Powprev) && (Vpanel <= Vpanel_prev))
    {
        if((CCR_Val - CCR_Val_step)<=235)
        {
            CCR_Val=235;
            DCDC_SetDutyCycle((u16)CCR_Val);
        }
        else
        {
            CCR_Val=CCR_Val - CCR_Val_step;
            DCDC_SetDutyCycle((u16)CCR_Val);
        }
    }

    if((Pow > Powprev) && (Vpanel > Vpanel_prev))
    {
        if((CCR_Val + CCR_Val_step)>=512)
        {
            CCR_Val=512;
            DCDC_SetDutyCycle((u16)CCR_Val);
        }
    else
        {   CCR_Val = CCR_Val + CCR_Val_step;
            DCDC_SetDutyCycle((u16)CCR_Val);
        }
    }

    if((Pow<Powprev) && (Vpanel<Vpanel_prev))
    {
        if((CCR_Val +CCR_Val_step)>=512)
        {
            CCR_Val=512;
            DCDC_SetDutyCycle((u16)CCR_Val);
        }
    else
        {
            CCR_Val = CCR_Val + CCR_Val_step;
            flag_mppt=1;
            DCDC_SetDutyCycle((u16)CCR_Val);
        }
    }

    if((Pow<=Powprev) && (Vpanel>=Vpanel_prev))
    {
        if((CCR_Val - CCR_Val_step)<=235)
        {
            CCR_Val=235;
            DCDC_SetDutyCycle((u16)CCR_Val);
        }
    else
        {
            CCR_Val = CCR_Val - CCR_Val_step;
            DCDC_SetDutyCycle((u16)CCR_Val);
        }
    }
    
    Powprev = Pow;
    Vpanel_prev= Vpanel;
    Pow=0;
}