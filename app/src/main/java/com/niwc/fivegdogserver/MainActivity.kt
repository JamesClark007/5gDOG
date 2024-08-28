package com.example.fivegdogserver

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.TextView
import android.widget.EditText
import android.widget.Button
import kotlinx.coroutines.*
import org.osmdroid.config.Configuration
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import com.example.fivegdogserver.databinding.ActivityMainBinding
import org.json.JSONObject
import android.graphics.Color

class MainActivity : AppCompatActivity(), CoroutineScope by MainScope() {
    private lateinit var binding: ActivityMainBinding
    private lateinit var map: MapView
    private lateinit var mapManager: MapManager
    private lateinit var gpsDataManager: GPSDataManager
    private lateinit var networkManager: NetworkManager
    private lateinit var infoOverlay: TextView
    private lateinit var statusOverlay: TextView
    private lateinit var gpsOverlay: TextView
    private lateinit var distanceToWaypointOverlay: TextView
    private lateinit var latitudeInput: EditText
    private lateinit var longitudeInput: EditText
    private lateinit var setWaypointButton: Button
    private val connectedClients = mutableListOf<String>()
    private var currentClientLocation: GeoPoint? = null
    private var currentWaypoint: GeoPoint? = null

    companion object {
        private const val TAG = "MainActivity"
        private const val SERVER_PORT = 8088
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Configuration.getInstance().load(applicationContext, getPreferences(MODE_PRIVATE))

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        map = binding.map
        infoOverlay = binding.infoOverlay
        statusOverlay = binding.statusOverlay
        gpsOverlay = binding.gpsOverlay
        distanceToWaypointOverlay = binding.distanceToWaypointOverlay
        latitudeInput = binding.latitudeInput
        longitudeInput = binding.longitudeInput
        setWaypointButton = binding.setWaypointButton

        mapManager = MapManager(this, map)
        mapManager.onWaypointUpdated = { geoPoint ->
            showSendWaypointButton(geoPoint)
        }
        gpsDataManager = GPSDataManager()
        networkManager = NetworkManager()

        binding.sendWaypointButton.setOnClickListener {
            mapManager.userWaypoint?.let { waypoint ->
                sendWaypointToClient(waypoint.position)
                hideSendWaypointButton()
            }
        }

        setWaypointButton.setOnClickListener {
            val latitude = latitudeInput.text.toString().toDoubleOrNull()
            val longitude = longitudeInput.text.toString().toDoubleOrNull()
            if (latitude != null && longitude != null) {
                val geoPoint = GeoPoint(latitude, longitude)
                mapManager.updateUserWaypoint(geoPoint)
                showSendWaypointButton(geoPoint)
            } else {
                updateStatusOverlay("Invalid coordinates", Color.RED)
            }
        }

        launch {
            try {
                Log.d(TAG, "Starting server on port $SERVER_PORT")
                networkManager.startServer(SERVER_PORT)
                updateInfoOverlay("Server started successfully")
                updateStatusOverlay("Server started", Color.GREEN)
                handleClientCommunication()
            } catch (e: Exception) {
                Log.e(TAG, "Failed to start server", e)
                val errorMessage = "Server error: ${e.message}\n${e.stackTraceToString()}"
                updateInfoOverlay(errorMessage)
                updateStatusOverlay("Server failed to start: ${e.message}", Color.RED)
            }
        }
    }

    private fun showSendWaypointButton(geoPoint: GeoPoint) {
        binding.sendWaypointButton.visibility = View.VISIBLE
        binding.sendWaypointButton.text = "Send Waypoint (${geoPoint.latitude.format(5)}, ${geoPoint.longitude.format(5)})"
        currentWaypoint = geoPoint
        updateDistanceToWaypoint()
    }

    private fun hideSendWaypointButton() {
        binding.sendWaypointButton.visibility = View.GONE
    }

    private fun sendWaypointToClient(geoPoint: GeoPoint) {
        val waypointJson = JSONObject().apply {
            put("waypoint", JSONObject().apply {
                put("lat", geoPoint.latitude)
                put("lon", geoPoint.longitude)
            })
        }
        launch {
            networkManager.sendToAllClients(waypointJson.toString())
            updateStatusOverlay("Sent waypoint to client", Color.BLUE)
            currentWaypoint = geoPoint
            updateDistanceToWaypoint()
        }
    }

    private fun updateInfoOverlay(additionalInfo: String = "") {
        val info = buildString {
            appendLine("Server: ${networkManager.getBoundAddress()}")
            appendLine("Clients: ${connectedClients.size}")
            connectedClients.forEach { appendLine(it) }
            if (additionalInfo.isNotEmpty()) {
                appendLine(additionalInfo)
            }
        }

        runOnUiThread {
            infoOverlay.text = info
        }
    }

    private fun updateStatusOverlay(message: String, color: Int) {
        runOnUiThread {
            statusOverlay.text = message
            statusOverlay.setTextColor(color)
        }
    }

    private fun updateGPSOverlay(geoPoint: GeoPoint, totalDistance: Double) {
        val info = buildString {
            appendLine("GPS Data:")
            appendLine("Latitude: ${geoPoint.latitude.format(5)}")
            appendLine("Longitude: ${geoPoint.longitude.format(5)}")
            appendLine("Total Distance: %.2f km".format(totalDistance))
        }

        runOnUiThread {
            gpsOverlay.text = info
        }
    }

    private fun updateDistanceToWaypoint() {
        val clientLocation = currentClientLocation
        val waypoint = currentWaypoint

        if (clientLocation != null && waypoint != null) {
            val distance = gpsDataManager.calculateDistanceToWaypoint(clientLocation, waypoint)
            runOnUiThread {
                distanceToWaypointOverlay.text = "Distance to Waypoint:\n%.2f meters\n%.2f feet".format(distance.first, distance.second)
                distanceToWaypointOverlay.visibility = View.VISIBLE
            }
        } else {
            runOnUiThread {
                distanceToWaypointOverlay.visibility = View.GONE
            }
        }
    }

    private suspend fun handleClientCommunication() {
        while (isActive) {
            try {
                val client = networkManager.acceptClient()
                client?.let {
                    val clientAddress = it.inetAddress.hostAddress
                    Log.d(TAG, "New client connected: $clientAddress")
                    connectedClients.add(clientAddress ?: "Unknown")
                    updateInfoOverlay()
                    updateStatusOverlay("New client connected: $clientAddress", Color.GREEN)
                    launch { handleClient(it) }
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error accepting client", e)
                updateStatusOverlay("Error accepting client", Color.RED)
            }
        }
    }

    private suspend fun handleClient(client: java.net.Socket) {
        try {
            while (isActive) {
                val data = networkManager.receiveData(client)
                if (data != null) {
                    Log.d(TAG, "Received data from client ${client.inetAddress.hostAddress}: $data")
                    updateStatusOverlay("Received data from client", Color.BLUE)
                    processClientData(data)
                } else {
                    Log.d(TAG, "Client ${client.inetAddress.hostAddress} disconnected")
                    connectedClients.remove(client.inetAddress.hostAddress)
                    updateInfoOverlay()
                    updateStatusOverlay("Client disconnected: ${client.inetAddress.hostAddress}", Color.YELLOW)
                    break
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error handling client ${client.inetAddress.hostAddress}", e)
            updateStatusOverlay("Error handling client", Color.RED)
        } finally {
            client.close()
            connectedClients.remove(client.inetAddress.hostAddress)
            updateInfoOverlay()
        }
    }

    private fun processClientData(data: String) {
        try {
            val jsonObject = JSONObject(data)
            when {
                jsonObject.has("gps") -> {
                    val gpsObject = jsonObject.getJSONObject("gps")
                    val lat = gpsObject.getDouble("lat")
                    val lon = gpsObject.getDouble("lon")
                    Log.d(TAG, "Received GPS data from client: lat=$lat, lon=$lon")
                    val geoPoint = GeoPoint(lat, lon)
                    currentClientLocation = geoPoint
                    val distanceTraveled = gpsDataManager.updateLocation(geoPoint)
                    mapManager.updateClientLocation(geoPoint)
                    updateGPSOverlay(geoPoint, gpsDataManager.totalDistanceTraveled)
                    updateDistanceToWaypoint()
                    updateStatusOverlay("Received GPS data", Color.GREEN)
                }
                jsonObject.has("waypoint") -> {
                    val waypointObject = jsonObject.getJSONObject("waypoint")
                    val lat = waypointObject.getDouble("lat")
                    val lon = waypointObject.getDouble("lon")
                    Log.d(TAG, "Received waypoint from client: lat=$lat, lon=$lon")
                    val geoPoint = GeoPoint(lat, lon)
                    currentWaypoint = geoPoint
                    mapManager.updateClientWaypoint(geoPoint)
                    updateDistanceToWaypoint()
                    updateStatusOverlay("Received waypoint from client", Color.MAGENTA)
                }
                else -> {
                    Log.d(TAG, "Received unknown data type from client: $data")
                    updateStatusOverlay("Received unknown data type", Color.YELLOW)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error processing client data: ${e.message}")
            updateStatusOverlay("Error processing client data", Color.RED)
        }
    }

    override fun onResume() {
        super.onResume()
        map.onResume()
    }

    override fun onPause() {
        super.onPause()
        map.onPause()
    }

    override fun onDestroy() {
        super.onDestroy()
        networkManager.close()
        cancel() // Cancel all coroutines when the activity is destroyed
    }

    private fun Double.format(digits: Int) = "%.${digits}f".format(this)
}

// add distance from waypoint to client
// add achieved waypoint sparkles or visual and audio indicator

// satellite terrain view option for map