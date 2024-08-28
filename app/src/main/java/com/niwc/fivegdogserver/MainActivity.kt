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
import android.text.SpannableString
import android.text.Spanned
import android.text.style.ForegroundColorSpan

class MainActivity : AppCompatActivity(), CoroutineScope by MainScope() {
    private lateinit var binding: ActivityMainBinding
    private lateinit var map: MapView
    private lateinit var mapManager: MapManager
    private lateinit var gpsDataManager: GPSDataManager
    private lateinit var networkManager: NetworkManager
    private lateinit var combinedOverlay: TextView
    private lateinit var latitudeInput: EditText
    private lateinit var longitudeInput: EditText
    private lateinit var setWaypointButton: Button
    private val connectedClients = mutableListOf<String>()
    private var currentClientLocation: GeoPoint? = null
    private var currentWaypoint: GeoPoint? = null
    private var lastStatusMessage = ""
    private var lastStatusColor = Color.WHITE
    private var lastGPSInfo = ""
    private var lastDistanceToWaypoint = ""

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
        combinedOverlay = binding.combinedOverlay
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
                updateCombinedOverlay(statusMessage = "Invalid coordinates", statusColor = Color.RED)
            }
        }

        launch {
            try {
                Log.d(TAG, "Starting server on port $SERVER_PORT")
                networkManager.startServer(SERVER_PORT)
                updateCombinedOverlay(statusMessage = "Server started", statusColor = Color.GREEN)
                handleClientCommunication()
            } catch (e: Exception) {
                Log.e(TAG, "Failed to start server", e)
                updateCombinedOverlay(statusMessage = "Server failed to start: ${e.message}", statusColor = Color.RED)
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
            updateCombinedOverlay(statusMessage = "Sent waypoint to client", statusColor = Color.BLUE)
            currentWaypoint = geoPoint
            updateDistanceToWaypoint()
        }
    }

    private fun updateCombinedOverlay(statusMessage: String = lastStatusMessage, statusColor: Int = lastStatusColor, gpsInfo: String = lastGPSInfo, distanceToWaypoint: String = lastDistanceToWaypoint) {
        lastStatusMessage = statusMessage
        lastStatusColor = statusColor
        lastGPSInfo = gpsInfo
        lastDistanceToWaypoint = distanceToWaypoint

        val combinedInfo = buildString {
            appendLine("Server: ${networkManager.getBoundAddress()}")
            appendLine("Clients: ${connectedClients.size}")
            connectedClients.forEach { appendLine(it) }
            appendLine("Status: $statusMessage")
            if (gpsInfo.isNotEmpty()) {
                appendLine(gpsInfo)
            }
            if (distanceToWaypoint.isNotEmpty()) {
                appendLine(distanceToWaypoint)
            }
            currentWaypoint?.let {
                appendLine("Current Waypoint: ${it.latitude.format(5)}, ${it.longitude.format(5)}")
            }
        }

        val spannableString = SpannableString(combinedInfo)
        val statusStart = combinedInfo.indexOf("Status: ")
        if (statusStart != -1) {
            val statusEnd = combinedInfo.indexOf('\n', statusStart)
            spannableString.setSpan(
                ForegroundColorSpan(statusColor),
                statusStart,
                if (statusEnd != -1) statusEnd else combinedInfo.length,
                Spanned.SPAN_EXCLUSIVE_EXCLUSIVE
            )
        }

        runOnUiThread {
            combinedOverlay.text = spannableString
        }
    }

    private fun updateDistanceToWaypoint() {
        val clientLocation = currentClientLocation
        val waypoint = currentWaypoint

        if (clientLocation != null && waypoint != null) {
            val distance = gpsDataManager.calculateDistanceToWaypoint(clientLocation, waypoint)
            lastDistanceToWaypoint = "Distance to Waypoint:\n%.2f meters\n%.2f feet".format(distance.first, distance.second)
            updateCombinedOverlay()
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
                    updateCombinedOverlay(statusMessage = "New client connected: $clientAddress", statusColor = Color.GREEN)
                    launch { handleClient(it) }
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error accepting client", e)
                updateCombinedOverlay(statusMessage = "Error accepting client", statusColor = Color.RED)
            }
        }
    }

    private suspend fun handleClient(client: java.net.Socket) {
        try {
            while (isActive) {
                val data = networkManager.receiveData(client)
                if (data != null) {
                    Log.d(TAG, "Received data from client ${client.inetAddress.hostAddress}: $data")
                    updateCombinedOverlay(statusMessage = "Received data from client", statusColor = Color.BLUE)
                    processClientData(data)
                } else {
                    Log.d(TAG, "Client ${client.inetAddress.hostAddress} disconnected")
                    connectedClients.remove(client.inetAddress.hostAddress)
                    updateCombinedOverlay(statusMessage = "Client disconnected: ${client.inetAddress.hostAddress}", statusColor = Color.YELLOW)
                    break
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error handling client ${client.inetAddress.hostAddress}", e)
            updateCombinedOverlay(statusMessage = "Error handling client", statusColor = Color.RED)
        } finally {
            client.close()
            connectedClients.remove(client.inetAddress.hostAddress)
            updateCombinedOverlay()
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
                    lastGPSInfo = buildString {
                        appendLine("GPS Data:")
                        appendLine("Latitude: ${lat.format(5)}")
                        appendLine("Longitude: ${lon.format(5)}")
                        appendLine("Total Distance: %.2f km".format(gpsDataManager.totalDistanceTraveled))
                    }
                    updateCombinedOverlay(statusMessage = "Received GPS data", statusColor = Color.GREEN)
                    updateDistanceToWaypoint()
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
                    updateCombinedOverlay(statusMessage = "Received waypoint from client", statusColor = Color.MAGENTA)
                }
                else -> {
                    Log.d(TAG, "Received unknown data type from client: $data")
                    updateCombinedOverlay(statusMessage = "Received unknown data type", statusColor = Color.YELLOW)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error processing client data: ${e.message}")
            updateCombinedOverlay(statusMessage = "Error processing client data", statusColor = Color.RED)
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