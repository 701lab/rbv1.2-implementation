#ifndef EXPLANATIONS_H_
#define EXPLANATIONS_H_

/*
	This document will contain only text information about ways chosen for library implementation

	1. SYSCLK frequency could be changed only by reprogramming. So no dynamic changes are allowed. The main reason is the big amount of time-dependent functions because of control theory implementations.
	 	 Every SYSCLK change should be carefully traced in all time-related setups.
	 	 Only frequencies multiple to 2 Mhz are allowed.

	2. All peripherals sets up in different functions. It makes code bigger, but easier to comment, understand and maintain.







	Протестировано:

	* Гибкая настройка частоты SYSCLK с шагом в 2Мгц работает как с HSI, так и с HSE
	* Переключение с HSI на HSE при отключении HSE работает. Частота работы сохраняется
	* Гибкая настройка частоты шима сохраняется
	* Гибкая настройка частоты UART работает в зависимости от частоты процессора

	* Гибкая настройка частоты прерывания SysTick работает

	Непротестировано:
	* Переключение на HSI при проблемах с PLL (не уверен, что вообще может такое произойти и тем более не знаю, как это протестировать)

 */





#endif /* EXPLANATIONS_H_ */
