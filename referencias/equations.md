# Equações

Força:

$$F = ma = m\ddot{x}$$  

$$ F = \frac{Ik}{r} = m\ddot{x} \longrightarrow  I = \frac{rm\ddot{x}}{k} $$  

Corrente:

$$ I = \frac{rm\ddot{x}}{2k} $$  


Velocidade angular:

$$ \omega = \frac{\dot{x}}{r} $$  

Equação do motor:

$$ V = RI + k\omega $$  

$$ V = R\frac{rm\ddot{x}}{2k} + k\frac{\dot{x}}{r} $$  

$$ V = (\frac{rRm}{2k})\ddot{x} + (\frac{k}{r})\dot{x} $$  

Laplace:

$$ V(s) = (\frac{rRm}{2k})X(s)s^2 + (\frac{k}{r})X(s)s $$  

$$ V(s) = X(s)\Big( \big(\frac{rRm}{2k}\big)s^2 + \big(\frac{k}{r}\big)s \Big) $$  

$$ \frac{X(s)}{V(s)} = \frac{1}{\big(\frac{rRm}{2k}\big)s^2 + \big(\frac{k}{r}\big)s} $$  

Equação de transferência do sistema:

$$ H(s) = \frac{X(s)}{V(s)} = \frac{ \Big(\frac{2k}{rRm}\Big) }{ s^2 + \big(\frac{2k^2}{Rmr^2}\big)s} $$  

onde:
* $F$ é a força nas rodas do robô
* $m$ é a massa do robô
* $a$ aceleração
* $x$ posição
* $r$ raio das rodas do robô
* $k$ constante do motor
* $kp$ constante proporcional do controlador PD
* $kd$ constante derivativa do controlador PD

## Laplace

![image](https://github.com/luisf18/Futebol_VSSS_F18/assets/36177710/c7abf6ea-3ef7-4ab3-b4d4-403136ec79fe)


## O controlador

$$ PD = \frac{OUT(s)}{ERRO(s)} = k_p\Big( 1 + sk_d \Big) $$  

## O controlador em malha fechada

$$ G_{open}(s) = PD(s)\times H(s) = k_p\Big( 1 + sk_d \Big) \times \frac{ \Big(\frac{2k}{rRm}\Big) }{ s^2 + \big(\frac{2k^2}{Rmr^2}\big)s} $$  

$$ G_{open}(s) = k_p\Big( 1 + sk_d \Big) \times \frac{ \Big(\frac{2k}{rRm}\Big) }{ s\big(s + \big(\frac{2k^2}{Rmr^2}\big) \big) } $$  

$$ G_{open}(s) = \frac{  k_p\Big( 1 + k_ds \Big) \Big(\frac{r}{k}\Big) }{ s\big( 1 + \big(\frac{Rmr^2}{2k^2}\big)s \big) } $$  

$$ G_{close}(s) = \frac{ G_{open} }{ 1 + G_{open} } $$  


impondo que o polo e o zero de equação de controle em malha aberta se anulem, segue:

$$ kd = \frac{Rmr^2}{2k^2} $$

$$ G_{open}(s) = \frac{  k_p\Big(\frac{r}{k}\Big) }{ s } = \frac{ \Big(\frac{k_pr}{k}\Big) }{ s } $$  

$$ G_{close}(s) = \frac{ 1 }{ \frac{ s }{ \Big(\frac{k_pr}{k}\Big) } + 1 } $$  

$$ G_{close}(s) = \frac{ \Big(\frac{k_pr}{k}\Big) }{ s + \Big(\frac{k_pr}{k}\Big) } $$  

$$  \frac{1}{\tau} = \frac{k_pr}{k} $$

$$ G_{close}(s) = \frac{ \frac{1}{\tau} }{ s + \frac{1}{\tau} } $$  

resposta ao degrau

$$ X_{set}(s) = \frac{ u }{ s } $$  

$$  X_{out}(s) = X_{set}(s) \times G_{close} = \frac{ u }{ s } \times \frac{ (\frac{1}{\tau}) }{ s + (\frac{1}{\tau}) }  $$   

$$  X_{out}(s) = u\Big( \frac{ 1 }{ s } + \frac{ -1 }{ s + \frac{1}{\tau} } \Big) $$   

$$  x_{out}(t) = u\big( 1 - e^{ \frac{-t}{\tau} } \big) $$   
  

## Resumindo o projeto:

$$  k_{p} = \frac{ k }{ r \tau } $$  

$$ k_{d} = \frac{Rmr^2}{2k^2} $$ 

## Facilitando as contas:

$$ \tau = \frac{N}{FPS} $$ 

$$ k = \frac{V_N}{\omega_n} = \frac{30V_N}{RPM \pi} $$ 

$$  k_{p} = \frac{ V_n }{ \omega_n r \tau } = \frac{ V_n FPS }{ \omega_n r N } $$  

$$ k_{d} = \frac{mr^2\omega_n^2 V_n}{2V_n^2 I_{stall}} = \frac{mr^2\omega_n^2}{2V_n I_{stall}} $$ 


    
