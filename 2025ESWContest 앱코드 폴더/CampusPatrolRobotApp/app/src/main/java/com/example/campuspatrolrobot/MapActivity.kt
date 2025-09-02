package com.example.campuspatrolrobot
import android.Manifest
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.viewinterop.AndroidView
import com.example.campuspatrolrobot.ui.theme.CampusPatrolRobotAppTheme
import com.google.android.gms.common.ConnectionResult
import com.google.android.gms.common.GoogleApiAvailability
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.MapView
import com.google.android.gms.maps.model.BitmapDescriptorFactory
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.maps.model.MapStyleOptions
import com.google.android.gms.maps.model.Marker
import com.google.android.gms.maps.model.MarkerOptions
import com.google.firebase.firestore.ktx.firestore
import com.google.firebase.ktx.Firebase
import com.google.firebase.firestore.Query
import android.util.Log
import java.text.SimpleDateFormat
import java.util.Locale
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import androidx.compose.runtime.rememberCoroutineScope
class MapActivity : ComponentActivity() {
    private lateinit var googleMapInstance: GoogleMap
    private val locationPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val fineLocationGranted = permissions[Manifest.permission.ACCESS_FINE_LOCATION] ?: false
        val coarseLocationGranted = permissions[Manifest.permission.ACCESS_COARSE_LOCATION] ?: false
        if (!fineLocationGranted || !coarseLocationGranted) {
            finish()
        }
    }
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val resultCode = GoogleApiAvailability.getInstance().isGooglePlayServicesAvailable(this)
        if (resultCode != ConnectionResult.SUCCESS) {
            if (GoogleApiAvailability.getInstance().isUserResolvableError(resultCode)) {
                GoogleApiAvailability.getInstance().getErrorDialog(this, resultCode, 1)?.show()
            }
            finish()
            return
        }
        locationPermissionLauncher.launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION
            )
        )
        setContent {
            CampusPatrolRobotAppTheme(darkTheme = true) {
                MapScreen()
            }
        }
    }
    @Composable
    fun MapScreen(modifier: Modifier = Modifier) {
        val context = LocalContext.current
        val mapView = remember { MapView(context) }
        var currentRobotMarker by remember { mutableStateOf<Marker?>(null) }
        val eventMarkers = remember { mutableListOf<Marker>() }
        var currentRobotState by remember { mutableStateOf("00") }
        var currentRobotStateTimestamp by remember { mutableStateOf("Unknown") }
        val firestore = Firebase.firestore
        var mapLoaded by remember { mutableStateOf(false) }
        val dateFormatter = remember { SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault()) }
        val coroutineScope = rememberCoroutineScope()
        LaunchedEffect(Unit) {
            mapView.onCreate(null)
            mapView.onStart()
            mapView.onResume()
        }
        DisposableEffect(Unit) {
            onDispose {
                mapView.onPause()
                mapView.onStop()
                mapView.onDestroy()
            }
        }
        AndroidView(
            factory = { mapView },
            modifier = modifier.fillMaxSize(),
            update = { map ->
                map.getMapAsync { googleMap ->
                    googleMapInstance = googleMap
                    mapLoaded = true
                    googleMap.uiSettings.isZoomControlsEnabled = true
                    googleMap.uiSettings.isMapToolbarEnabled = true
                    googleMap.setMapStyle(MapStyleOptions.loadRawResourceStyle(context, R.raw.map_style_dark))
                    val initialLatLng = LatLng(37.5665, 126.9780)
                    if (currentRobotMarker == null) {
                        currentRobotMarker = googleMap.addMarker(
                            MarkerOptions()
                                .position(initialLatLng)
                                .title("Robot Initial Location")
                                .snippet("")
                                .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_CYAN))
                        )
                        Log.d("MapActivity", "Initial robot marker added.")
                    } else {
                        currentRobotMarker?.position = initialLatLng
                        currentRobotMarker?.title = "Robot Initial Location"
                        currentRobotMarker?.snippet = ""
                    }
                    googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(initialLatLng, 16f))
                    Log.d("MapActivity", "Map initialized and camera moved to initial position with zoom 16f.")
                }
            }
        )
        LaunchedEffect(mapLoaded) {
            if (!mapLoaded) {
                Log.d("MapActivity", "Map not loaded yet, skipping Robot_Current_Location listener setup.")
                return@LaunchedEffect
            }
            Log.d("MapActivity", "Map loaded, setting up Robot_Current_Location listener.")
            firestore.collection("Robot_Current_Location")
                .document("robot_1")
                .addSnapshotListener { snapshot, e ->
                    if (e != null) {
                        Log.e("MapActivity", "Error listening to Robot_Current_Location: ${e.message}")
                        return@addSnapshotListener
                    }
                    snapshot?.data?.let { data ->
                        val lat = data["latitude"] as? Double ?: 0.0
                        val lng = data["longitude"] as? Double ?: 0.0
                        val timestamp = data["timestamp"] as? com.google.firebase.Timestamp
                        val gpsTime = timestamp?.toDate()?.let { dateFormatter.format(it) } ?: "Unknown"
                        val newLatLng = LatLng(lat, lng)
                        Log.d("MapActivity", "Received Robot_Current_Location: Lat=$lat, Lng=$lng, Time=$gpsTime")
                        if (currentRobotMarker == null) {
                            currentRobotMarker = googleMapInstance.addMarker(
                                MarkerOptions()
                                    .position(newLatLng)
                                    .title("Robot Location")
                                    .snippet("")
                                    .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_CYAN))
                            )
                            Log.d("MapActivity", "currentRobotMarker was null, created new one.")
                        } else {
                            currentRobotMarker?.position = newLatLng
                            currentRobotMarker?.title = "Robot Location"
                            currentRobotMarker?.snippet = ""
                            Log.d("MapActivity", "Updated currentRobotMarker position and title.")
                        }
                        val stateText = when (currentRobotState) {
                            "00" -> "Patrolling"
                            "01" -> "Patrol Complete"
                            "10" -> "Kickboard Detected"
                            "11" -> "Error Occurred"
                            else -> "Unknown Status"
                        }
                        if (currentRobotMarker?.isInfoWindowShown != true) {
                            currentRobotMarker?.showInfoWindow()
                        }
                        val currentZoom = googleMapInstance.cameraPosition.zoom
                        googleMapInstance.animateCamera(
                            CameraUpdateFactory.newLatLngZoom(newLatLng, currentZoom),
                            1000,
                            null
                        )
                        Log.d("MapActivity", "Camera animated to new robot location with current zoom: $currentZoom")
                    } ?: run {
                        Log.d("MapActivity", "Robot_Current_Location snapshot data is null.")
                    }
                }
        }
        LaunchedEffect(Unit) {
            firestore.collection("Robot_State")
                .document("current_state")
                .addSnapshotListener { snapshot, e ->
                    if (e != null) {
                        Log.e("MapActivity", "Error listening to Robot_State: ${e.message}")
                        return@addSnapshotListener
                    }
                    snapshot?.data?.let { data ->
                        val newState = data["state"] as? String ?: "00"
                        val newTimestamp = data["timestamp"] as? com.google.firebase.Timestamp
                        currentRobotState = newState
                        currentRobotStateTimestamp = newTimestamp?.toDate()?.let { dateFormatter.format(it) } ?: "Unknown"
                        Log.d("MapActivity", "Received Robot_State: State=$newState, Timestamp=$currentRobotStateTimestamp")
                        val stateText = when (currentRobotState) {
                            "00" -> "Patrolling"
                            "01" -> "Patrol Complete"
                            "10" -> "Kickboard Detected"
                            "11" -> "Error Occurred"
                            else -> "Unknown Status"
                        }
                        val currentGPSSnippet = currentRobotMarker?.snippet
                        val gpsTimeFromSnippet = if (currentGPSSnippet != null && "GPS Time:" in currentGPSSnippet) {
                            currentGPSSnippet.substringAfter("GPS Time: ").trim()
                        } else {
                            "Unknown"
                        }
                        currentRobotMarker?.snippet = ""
                        if (currentRobotMarker?.isInfoWindowShown != true) {
                            currentRobotMarker?.showInfoWindow()
                        }
                        Log.d("MapActivity", "Updated robot marker snippet with new state.")
                        // Revert to Patrolling ("00") after 1 second
                        coroutineScope.launch {
                            delay(1000) // 1-second delay
                            if (currentRobotState == "10") {
                                currentRobotState = "00"
                                val newTimestampAfterDelay = com.google.firebase.Timestamp.now()
                                currentRobotStateTimestamp = dateFormatter.format(newTimestampAfterDelay.toDate())
                                val defaultStateText = "Patrolling"
                                currentRobotMarker?.snippet = ""
                                if (currentRobotMarker?.isInfoWindowShown != true) {
                                    currentRobotMarker?.showInfoWindow()
                                }
                                Log.d("MapActivity", "Reverted robot state to Patrolling (00) after 1-second delay.")
                            }
                        }
                    } ?: run {
                        Log.d("MapActivity", "Robot_State snapshot data is null.")
                    }
                }
        }
        LaunchedEffect(Unit) {
            firestore.collection("Event_Gps_Data")
                .orderBy("timestamp", Query.Direction.DESCENDING)
                .limit(20)
                .addSnapshotListener { snapshot, e ->
                    if (e != null) {
                        Log.e("MapActivity", "Error listening to Event_Gps_Data: ${e.message}")
                        return@addSnapshotListener
                    }
                    eventMarkers.forEach { it.remove() }
                    eventMarkers.clear()
                    Log.d("MapActivity", "Cleared existing event markers for Event_Gps_Data.")
                    snapshot?.documents?.forEachIndexed { index, document ->
                        val data = document.data
                        val lat = data?.get("latitude") as? Double ?: return@forEachIndexed
                        val lng = data["longitude"] as? Double ?: return@forEachIndexed
                        val eventType = data["eventType"] as? String ?: "Unknown Event"
                        val eventState = data["state"] as? String ?: "Unknown"
                        val timestamp = data["timestamp"] as? com.google.firebase.Timestamp
                        if (eventState == "10") {
                            val eventTime = timestamp?.toDate()?.let { dateFormatter.format(it) } ?: "Unknown"
                            val eventLatLng = LatLng(lat, lng)
                            val eventId = index + 1
                            val eventMarker = googleMapInstance.addMarker(
                                MarkerOptions()
                                    .position(eventLatLng)
                                    .title("Kickboard Detected #$eventId")
                                    .snippet("Time: $eventTime\nID: $eventId")
                                    .icon(BitmapDescriptorFactory.defaultMarker(20f))
                                    .alpha(0.8f)
                            )
                            eventMarker?.showInfoWindow()
                            if (eventMarker != null) {
                                eventMarkers.add(eventMarker)
                            }
                            Log.d("MapActivity", "Added event marker from Event_Gps_Data: $eventType (State 10, ID=$eventId) at ($lat, $lng)")
                        } else {
                            Log.d("MapActivity", "Skipped event marker for state: $eventState at ($lat, $lng)")
                        }
                    } ?: run {
                        Log.d("MapActivity", "Event_Gps_Data snapshot data is null.")
                    }
                }
        }
    }
}
