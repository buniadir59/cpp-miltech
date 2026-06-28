#pragma once

/* TODO
клас фізики дрона . Володіє станом дрона, приймає команди через чергу і у власному потоці кожні physicsTimeStep інтегрує рух.

struct DroneCommand {
	DroneState state;   // новий режим
	float angleSpeed;  	// Кутова швидкість повороту
};
 
struct DroneTelemetry {
	Coord pos;
	Coord speed;
	float timeSecSinceStart;
};

 timeSecSinceStart - це час останнього оновлення значень фізики. Потрібно для того, щоб компенсувати нерівномірність 
 кроків, яка виникне при збереженні аутпуту
 */