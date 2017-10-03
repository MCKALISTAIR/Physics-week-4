 #include <iostream >
 #include <cmath >
 #include "Force.h"
 #include "Body.h"
 #include "glm/ext.hpp"

 glm::vec3 Force::apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel) {
	 return glm::vec3(0.0f);
	
}

 /*
   12 ** GRAVITY
   13 */
	 glm::vec3 Gravity::apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel) {
	 // complete. Should return the acceleration resulting from gravity
		 return m_gravity * mass;
		
}

 /*
   19 ** DRAG
   20 */
	 glm::vec3 Gravity::apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel) {
 // complete. Should return the acceleration resulting from aerodynamic drag
		return 0.5 * (mass * vel )
}