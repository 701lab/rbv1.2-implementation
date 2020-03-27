#ifndef EXPLANATIONS_H_
#define EXPLANATIONS_H_

/*
	This document will contain only text information about ways chosen for library implementation

	1. SYSCLK frequency could be changed only by reprogramming. So no dynamic changes are allowed. The main reason is the big amount of time-dependent functions because of control theory implementations.
	 	 Every SYSCLK change should be carefully traced in all time-related setups.
	 	 Only frequencies multiple to 2 Mhz are allowed.

	2. All peripherals sets up in different functions. It makes code bigger, but easier to comment, understand and maintain.







	Протестировано:

	1. Гибкая настройка частоты с использованием макроса и

 */





#endif /* EXPLANATIONS_H_ */
