package com.example.campuspatrolrobot

import android.content.Context
import android.content.Intent
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.animation.animateContentSize
import androidx.compose.animation.core.spring
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.scale
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.RectangleShape
import androidx.compose.ui.graphics.painter.Painter
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.dp
import com.example.campuspatrolrobot.ui.theme.CampusPatrolRobotAppTheme
import com.google.firebase.firestore.ktx.firestore
import com.google.firebase.ktx.Firebase
import java.text.SimpleDateFormat
import java.util.Locale
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.delay
import androidx.core.net.toUri
import androidx.core.content.edit

class ChatActivity2 : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            CampusPatrolRobotAppTheme(darkTheme = true) {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    EventScreen()
                }
            }
        }
    }
}

// Save event data to SharedPreferences
fun saveEvents(context: Context, events: List<EventData>) {
    val prefs = context.getSharedPreferences("EventLogs", Context.MODE_PRIVATE)
    val serializedEvents = events.map { "${it.timestamp}|${it.eventType}|${it.latitude}|${it.longitude}|${it.docId}" }.toSet()
    prefs.edit { putStringSet("logs", serializedEvents) }
}

// Clear event data from SharedPreferences and Firestore
fun clearEvents(context: Context) {
    val prefs = context.getSharedPreferences("EventLogs", Context.MODE_PRIVATE)
    prefs.edit { remove("logs") }

    val firestore = Firebase.firestore
    firestore.collection("Event_Gps_Data")
        .get()
        .addOnSuccessListener { querySnapshot ->
            val batch = firestore.batch()
            querySnapshot.documents.forEach { document ->
                batch.delete(document.reference)
            }
            batch.commit()
        }
}

// Event data class
data class EventData(
    val timestamp: String,
    val eventType: String,
    val latitude: Double,
    val longitude: Double,
    val docId: String
)

// Event history screen
@Composable
fun EventScreen(modifier: Modifier = Modifier) {
    val events = remember { mutableStateListOf<EventData>() }
    val context = LocalContext.current
    var showClearDialog by remember { mutableStateOf(false) }
    val firestore = Firebase.firestore
    // lastEventTime을 Map<String, Long>으로 변경하여 각 상태별로 마지막 이벤트 시간을 추적
    val lastEventTimeMap = remember { mutableStateOf(mutableMapOf<String, Long>()) }

    LaunchedEffect(Unit) {
        val prefs = context.getSharedPreferences("EventLogs", Context.MODE_PRIVATE)
        val savedLogs = prefs.getStringSet("logs", emptySet())?.toList() ?: emptyList()
        savedLogs.forEach { log ->
            val parts = log.split("|")
            if (parts.size == 5) {
                try {
                    events.add(EventData(parts[0], parts[1], parts[2].toDouble(), parts[3].toDouble(), parts[4]))
                } catch (_: NumberFormatException) {
                }
            }
        }
        events.sortByDescending { it.timestamp }

        // Robot_State monitoring (for 01, 11)
        firestore.collection("Robot_State")
            .document("current_state")
            .addSnapshotListener { snapshot, e ->
                if (e != null) {
                    android.util.Log.e("Firestore", "Snapshot listener error: $e")
                    return@addSnapshotListener
                }
                snapshot?.data?.let { data ->
                    val state = data["state"] as? String ?: run {
                        android.util.Log.w("Firestore", "State is null or not a String")
                        return@let
                    }
                    android.util.Log.d("Firestore", "Received state from Robot_State: $state")
                    val timestampObject = data["timestamp"]
                    val formattedTimestamp = when (timestampObject) {
                        is com.google.firebase.Timestamp -> timestampObject.toDate().let {
                            SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault()).format(it)
                        } ?: ""
                        is String -> timestampObject.takeIf { it.isNotEmpty() } ?: ""
                        else -> {
                            android.util.Log.w("Firestore", "Invalid timestamp format")
                            ""
                        }
                    }
                    val currentTime = System.currentTimeMillis()
                    val lastTimeForState = lastEventTimeMap.value[state] ?: 0L

                    // 3초 중복 방지 로직을 각 상태별로 적용
                    if (currentTime - lastTimeForState < 3000) {
                        android.util.Log.d("EventScreen", "Skipping duplicate event for state $state within 3 seconds.")
                        return@let
                    }

                    val lat = data["latitude"] as? Double ?: run {
                        android.util.Log.w("Firestore", "Latitude is null or not a Double")
                        0.0
                    }
                    val lng = data["longitude"] as? Double ?: run {
                        android.util.Log.w("Firestore", "Longitude is null or not a Double")
                        0.0
                    }
                    val eventType = when (state) {
                        "00" -> "Unknown State" // "Patrolling"은 이벤트 히스토리에서 제외되므로 "Unknown State"로 매핑하거나 필요시 다른 처리를 할 수 있습니다.
                        "01" -> "Patrol Complete"
                        "11" -> "Problem Detected"
                        else -> "Unknown State"
                    }
                    val docId = "robot_state_${state}_${formattedTimestamp.replace(" ", "_").replace(":", "_")}"
                    val newEvent = EventData(formattedTimestamp, eventType, lat, lng, docId)

                    // '10' 상태와 '00' (Patrolling) 상태는 이벤트 히스토리에서 제외
                    if (state != "10" && state != "00" && !events.any { it.docId == newEvent.docId }) {
                        android.util.Log.d("EventScreen", "Adding new event from Robot_State: $newEvent")
                        events.add(0, newEvent)
                        saveEvents(context, events.toList())
                        lastEventTimeMap.value[state] = currentTime // 해당 상태의 마지막 이벤트 시간 업데이트
                    } else if (state == "10") {
                        android.util.Log.d("EventScreen", "Skipping state '10' as it's handled by Event_Gps_Data listener.")
                    } else if (state == "00") {
                        android.util.Log.d("EventScreen", "Skipping state '00' (Patrolling) as it's considered meaningless for event history.")
                    } else {
                        android.util.Log.d("EventScreen", "Event already exists or is duplicate for state $state: $docId")
                    }
                }
            }

        // Monitor Event_Gps_Data (for 10)
        firestore.collection("Event_Gps_Data")
            .orderBy("timestamp", com.google.firebase.firestore.Query.Direction.DESCENDING)
            .addSnapshotListener { snapshot, e ->
                if (e != null) {
                    android.util.Log.e("Firestore", "Event_Gps_Data listener error: $e")
                    return@addSnapshotListener
                }
                snapshot?.documentChanges?.forEach { change ->
                    if (change.type == com.google.firebase.firestore.DocumentChange.Type.ADDED) {
                        val data = change.document.data
                        val state = data["state"] as? String ?: "unknown"
                        android.util.Log.d("Firestore", "Received state from Event_Gps_Data: $state")
                        if (state != "10") {
                            android.util.Log.d("EventScreen", "Skipping non-'10' state from Event_Gps_Data: $state")
                            return@forEach // Only process '10' state
                        }
                        val timestampObject = data["timestamp"]
                        val formattedTimestamp = when (timestampObject) {
                            is com.google.firebase.Timestamp -> timestampObject.toDate().let {
                                SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault()).format(it)
                            }
                            is String -> timestampObject.takeIf { it.isNotEmpty() } ?: ""
                            else -> {
                                android.util.Log.w("Firestore", "Invalid timestamp format in Event_Gps_Data")
                                ""
                            }
                        }
                        val lat = data["latitude"] as? Double ?: run {
                            android.util.Log.w("Firestore", "Latitude is null or not a Double in Event_Gps_Data")
                            0.0
                        }
                        val lng = data["longitude"] as? Double ?: run {
                            android.util.Log.w("Firestore", "Longitude is null or not a Double in Event_Gps_Data")
                            0.0
                        }
                        val eventType = "Kickboard Detected"
                        val docId = change.document.id

                        val newEvent = EventData(formattedTimestamp, eventType, lat, lng, docId)
                        if (!events.any { it.docId == newEvent.docId }) {
                            android.util.Log.d("EventScreen", "Adding new event from Event_Gps_Data: $newEvent")
                            events.add(0, newEvent)
                            saveEvents(context, events.toList())
                        } else {
                            android.util.Log.d("EventScreen", "Event already exists: $docId")
                        }
                    }
                }
            }
    }

    if (showClearDialog) {
        AlertDialog(
            onDismissRequest = { showClearDialog = false },
            title = { Text("Clear Logs") },
            text = { Text("Do you want to delete all chat logs?") },
            confirmButton = {
                Button(onClick = {
                    clearEvents(context)
                    events.clear()
                    showClearDialog = false
                }) {
                    Text("Delete")
                }
            },
            dismissButton = {
                Button(onClick = { showClearDialog = false }) {
                    Text("Cancel")
                }
            }
        )
    }

    Column(modifier = modifier.padding(16.dp)) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Text(
                text = "EVENT HISTORY",
                style = MaterialTheme.typography.headlineMedium,
                color = MaterialTheme.colorScheme.onBackground
            )
            ClearLogButtonInEventScreen { showClearDialog = true }
        }
        HorizontalDivider(
            modifier = Modifier
                .fillMaxWidth()
                .padding(vertical = 8.dp),
            thickness = 1.dp,
            color = Color.Gray
        )
        Spacer(modifier = Modifier.height(16.dp))
        LazyColumn(
            modifier = Modifier.fillMaxSize(),
            reverseLayout = true
        ) {
            items(events.size) { index ->
                val event = events[index]
                EventItem(event = event)
            }
        }
    }
}

// Clear log button (with animation)
@Composable
fun ClearLogButtonInEventScreen(onClick: () -> Unit) {
    var isPressed by remember { mutableStateOf(false) }
    val animatedScale by animateFloatAsState(
        targetValue = if (isPressed) 0.95f else 1f,
        animationSpec = spring(),
        label = "clearLogButtonScaleAnimation"
    )

    Surface(
        modifier = Modifier
            .scale(animatedScale)
            .clickable(
                onClick = {
                    isPressed = true
                    CoroutineScope(Dispatchers.Main).launch {
                        delay(100)
                        onClick()
                        isPressed = false
                    }
                }
            )
            .animateContentSize(animationSpec = spring()),
        shape = RectangleShape,
        color = Color(0xFF666666),
        tonalElevation = 4.dp
    ) {
        Text(
            text = "Clear Log",
            modifier = Modifier.padding(horizontal = 16.dp, vertical = 8.dp),
            color = Color.White,
            style = MaterialTheme.typography.bodyMedium
        )
    }
}

// Individual event item (with map link)
@Composable
fun EventItem(event: EventData) {
    var isPressed by remember { mutableStateOf(false) }
    val animatedScale by animateFloatAsState(
        targetValue = if (isPressed) 0.95f else 1f,
        animationSpec = spring(),
        label = "eventItemScaleAnimation"
    )

    val context = LocalContext.current

    val eventIconPainter: Painter = when (event.eventType) {
        "Patrolling" -> painterResource(id = R.drawable.ic_patrol_complete)
        "Patrol Complete" -> painterResource(id = R.drawable.ic_patrol_complete)
        "Kickboard Detected" -> painterResource(id = R.drawable.ic_kickboard)
        "Problem Detected" -> painterResource(id = R.drawable.ic_problem)
        else -> painterResource(id = R.drawable.ic_notification)
    }

    Surface(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp)
            .scale(animatedScale)
            .clickable {
                isPressed = true
                CoroutineScope(Dispatchers.Main).launch {
                    delay(300)
                    isPressed = false

                    val mapUri =
                        "geo:${event.latitude},${event.longitude}?q=${event.latitude},${event.longitude}(${event.eventType})&z=16".toUri()
                    val mapIntent = Intent(Intent.ACTION_VIEW, mapUri)
                    mapIntent.setPackage("com.google.android.apps.maps")
                    try {
                        context.startActivity(mapIntent)
                    } catch (_: Exception) {
                        val webMapUri =
                            "http://maps.google.com/maps?q=${event.latitude},${event.longitude}".toUri()
                        val webIntent = Intent(Intent.ACTION_VIEW, webMapUri)
                        context.startActivity(webIntent)
                    }
                }
            }
            .animateContentSize(animationSpec = spring()),
        shape = RoundedCornerShape(16.dp),
        color = MaterialTheme.colorScheme.surface,
        tonalElevation = 2.dp
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .border(
                    width = 0.5.dp,
                    color = Color.White,
                    shape = RoundedCornerShape(16.dp)
                )
                .padding(12.dp)
                .widthIn(max = 250.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Column(
                modifier = Modifier.weight(1f)
            ) {
                Text(
                    text = event.timestamp,
                    color = Color.Gray,
                    style = MaterialTheme.typography.bodySmall,
                )
                Text(
                    text = event.eventType,
                    color = MaterialTheme.colorScheme.onSurface,
                    style = MaterialTheme.typography.bodyMedium,
                    modifier = Modifier.padding(top = 4.dp)
                )
            }
            Icon(
                painter = eventIconPainter,
                contentDescription = "${event.eventType} icon",
                tint = Color.Unspecified,
                modifier = Modifier.size(24.dp)
            )
        }
    }
}
