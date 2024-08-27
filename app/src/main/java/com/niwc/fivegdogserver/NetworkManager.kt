package com.example.fivegdogserver

import android.util.Log
import java.net.*
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.PrintWriter
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

class NetworkManager {
    private var serverSocket: ServerSocket? = null
    private val clients = mutableListOf<Socket>()

    companion object {
        private const val TAG = "NetworkManager"
    }

    suspend fun startServer(port: Int) = withContext(Dispatchers.IO) {
        try {
            // Create a server socket that binds to all IPv6 addresses
            val serverAddress = Inet6Address.getByName("::")
            serverSocket = ServerSocket()
            serverSocket?.bind(InetSocketAddress(serverAddress, port))

            val boundAddress = serverSocket?.inetAddress
            val boundPort = serverSocket?.localPort
            Log.i(TAG, "Server successfully bound and listening on [${boundAddress}]:$boundPort")

            // Check if we're actually using IPv6
            if (boundAddress is Inet6Address) {
                Log.i(TAG, "Server is using IPv6")
            } else {
                Log.w(TAG, "Server is not using IPv6 as expected")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start server on port $port", e)
            throw e
        }
    }

    suspend fun acceptClient(): Socket? = withContext(Dispatchers.IO) {
        try {
            Log.d(TAG, "Waiting for client connection...")
            val socket = serverSocket?.accept()
            socket?.let {
                clients.add(it)
                Log.d(TAG, "New client connected: [${it.inetAddress.hostAddress}]:${it.port}")
                it
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error accepting client", e)
            null
        }
    }

    suspend fun receiveData(client: Socket): String? = withContext(Dispatchers.IO) {
        try {
            val reader = BufferedReader(InputStreamReader(client.getInputStream()))
            val data = reader.readLine()
            Log.d(TAG, "Received data from [${client.inetAddress.hostAddress}]:${client.port}: $data")
            data
        } catch (e: Exception) {
            Log.e(TAG, "Error receiving data from [${client.inetAddress.hostAddress}]:${client.port}", e)
            null
        }
    }

    suspend fun sendData(client: Socket, data: String) = withContext(Dispatchers.IO) {
        try {
            val writer = PrintWriter(client.getOutputStream(), true)
            writer.println(data)
            Log.d(TAG, "Sent data to [${client.inetAddress.hostAddress}]:${client.port}: $data")
        } catch (e: Exception) {
            Log.e(TAG, "Error sending data to [${client.inetAddress.hostAddress}]:${client.port}", e)
        }
    }

    suspend fun sendToAllClients(data: String) = withContext(Dispatchers.IO) {
        clients.toList().forEach { client ->
            try {
                sendData(client, data)
            } catch (e: Exception) {
                Log.e(TAG, "Error sending data to client [${client.inetAddress.hostAddress}]:${client.port}: ${e.message}")
                clients.remove(client)
            }
        }
    }

    fun close() {
        serverSocket?.let { socket ->
            if (!socket.isClosed) {
                try {
                    socket.close()
                    Log.i(TAG, "Server socket closed")
                } catch (e: Exception) {
                    Log.e(TAG, "Error closing server socket", e)
                }
            }
        }
        clients.forEach { client ->
            try {
                client.close()
                Log.d(TAG, "Closed connection to [${client.inetAddress.hostAddress}]:${client.port}")
            } catch (e: Exception) {
                Log.e(TAG, "Error closing client socket [${client.inetAddress.hostAddress}]:${client.port}", e)
            }
        }
        clients.clear()
        Log.i(TAG, "All connections closed")
    }

    fun isServerRunning(): Boolean {
        return serverSocket?.isBound == true && !serverSocket?.isClosed!!
    }

    fun getBoundAddress(): String {
        return serverSocket?.let { "[${it.inetAddress.hostAddress}]:${it.localPort}" } ?: "Not bound"
    }
}