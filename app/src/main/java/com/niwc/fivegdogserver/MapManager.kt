package com.example.fivegdogserver

import android.content.Context
import android.view.MotionEvent
import androidx.core.content.ContextCompat
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Marker
import org.osmdroid.views.overlay.Overlay
import org.osmdroid.tileprovider.tilesource.TileSourceFactory

class MapManager(private val context: Context, private val map: MapView) {

    var userWaypoint: Marker? = null
        private set
    var clientLocationMarker: Marker? = null
        private set

    var onWaypointUpdated: ((GeoPoint) -> Unit)? = null

    init {
        setupMap()
    }

    private fun setupMap() {
        map.setTileSource(TileSourceFactory.MAPNIK)
        map.controller.setZoom(10.0)
        val oahuCenter = GeoPoint(21.4389, -158.0001)
        map.controller.setCenter(oahuCenter)  // This line moves the camera to Oahu

        map.overlays.add(object : Overlay() {
            override fun onSingleTapConfirmed(e: MotionEvent, mapView: MapView): Boolean {
                val projection = mapView.projection
                val geoPoint = projection.fromPixels(e.x.toInt(), e.y.toInt())
                updateUserWaypoint(GeoPoint(geoPoint.latitude, geoPoint.longitude))
                return true
            }
        })
    }

    fun updateUserWaypoint(geoPoint: GeoPoint) {
        if (userWaypoint == null) {
            userWaypoint = Marker(map).apply {
                icon = ContextCompat.getDrawable(context, R.drawable.ic_location_on_blue_24dp)
                setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
            }
            map.overlays.add(userWaypoint)
        }
        userWaypoint?.position = geoPoint
        moveToWaypointAndZoom(geoPoint)
        map.invalidate()
        onWaypointUpdated?.invoke(geoPoint)
    }

    fun updateClientWaypoint(geoPoint: GeoPoint) {
        updateUserWaypoint(geoPoint)
    }

    private fun moveToWaypointAndZoom(geoPoint: GeoPoint) {
        map.controller.animateTo(geoPoint)  // This line animates the camera to the waypoint
        map.controller.setCenter(geoPoint)
        map.controller.setZoom(18.0)
        map.controller.zoomTo(18.0, 1000L) // Zoom over 1 second
        println(geoPoint)
    }

    fun updateClientLocation(geoPoint: GeoPoint) {
        if (clientLocationMarker == null) {
            clientLocationMarker = Marker(map).apply {
                icon = ContextCompat.getDrawable(context, R.drawable.ic_location_on_red_24dp)
                setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
            }
            map.overlays.add(clientLocationMarker)
        }
        clientLocationMarker?.position = geoPoint
        moveToWaypointAndZoom(geoPoint)
        map.invalidate()
    }
}