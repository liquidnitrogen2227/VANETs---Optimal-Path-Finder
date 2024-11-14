//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.car2x.veins.modules.application.traci; // Updated package path

import org.car2x.veins.base.modules.IBaseApplLayer;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple CAMOACORouting extends DemoBaseApplLayer
{
    parameters:
        @class(CAMOACORouting);
        @display("i=block/app2");
        
        // CAMO-ACO parameters
        double evaporationRate = default(0.1);
        double alpha = default(1.0);
        double beta = default(2.0);
        int numAnts = default(10);
        
        // Routing update parameters
        double updateInterval @unit(s) = default(1s);
        
        @signal[routeUpdate](type=long);
        @statistic[routeUpdates](title="Number of route updates"; source="routeUpdate"; record=count,vector);
}
