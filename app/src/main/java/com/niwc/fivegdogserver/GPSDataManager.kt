package com.example.fivegdogserver

import org.osmdroid.util.GeoPoint
import kotlin.math.*

class GPSDataManager {
    private var lastClientLocation: GeoPoint? = null
    var totalDistanceTraveled = 0.0
        private set

    companion object {
        private const val EARTH_RADIUS_KM = 6371.0
    }

    fun updateLocation(newLocation: GeoPoint): Double {
        var distanceTraveled = 0.0
        lastClientLocation?.let { lastLocation ->
            distanceTraveled = calculateDistance(lastLocation, newLocation)
            totalDistanceTraveled += distanceTraveled
        }
        lastClientLocation = newLocation
        return distanceTraveled
    }

    private fun calculateDistance(start: GeoPoint, end: GeoPoint): Double {
        // Haversine formula to calculate distance between two points on a sphere
        val dLat = Math.toRadians(end.latitude - start.latitude)
        val dLon = Math.toRadians(end.longitude - start.longitude)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(start.latitude)) * cos(Math.toRadians(end.latitude)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return EARTH_RADIUS_KM * c
    }
}