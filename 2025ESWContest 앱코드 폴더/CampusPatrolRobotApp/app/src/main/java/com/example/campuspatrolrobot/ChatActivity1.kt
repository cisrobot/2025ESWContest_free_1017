package com.example.campuspatrolrobot

import android.content.Context
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.animation.animateContentSize
import androidx.compose.animation.core.spring
import androidx.compose.foundation.background
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
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.Send
import androidx.compose.material.icons.filled.CheckCircle
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.material3.TextFieldDefaults
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
import androidx.compose.ui.unit.dp
import com.example.campuspatrolrobot.ui.theme.CampusPatrolRobotAppTheme
import com.google.firebase.firestore.ktx.firestore
import com.google.firebase.ktx.Firebase
import java.text.SimpleDateFormat
import java.util.Locale
import androidx.core.content.edit

// Main activity for initializing the chat UI
class ChatActivity1 : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            CampusPatrolRobotAppTheme(darkTheme = true) {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    ChatScreen()
                }
            }
        }
    }
}

// Main chat screen composable managing messages and Firebase
@Composable
fun ChatScreen(modifier: Modifier = Modifier) {
    // State for message input and dialog
    var message by remember { mutableStateOf("") }
    val messages = remember { mutableStateListOf<Triple<String, String, String>>() }
    val context = androidx.compose.ui.platform.LocalContext.current
    var showClearDialog by remember { mutableStateOf(false) }

    // Fetch messages from Firestore
    LaunchedEffect(Unit) {
        val db = Firebase.firestore
        db.collection("Messages")
            .orderBy("timestamp")
            .addSnapshotListener { snapshot, _ ->
                snapshot?.documentChanges?.forEach { docChange ->
                    if (docChange.type == com.google.firebase.firestore.DocumentChange.Type.ADDED) {
                        val doc = docChange.document
                        val text = doc.getString("text") ?: ""
                        val sender = doc.getString("sender") ?: ""
                        val timestamp = doc.getTimestamp("timestamp")?.toDate()?.let {
                            SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.getDefault()).format(it)
                        } ?: ""
                        if (!messages.any { it.first == text && it.second == sender && it.third == timestamp }) {
                            messages.add(Triple(text, sender, timestamp))
                        }
                    }
                }
            }
    }

    // Send message to Firestore
    fun sendMessage(text: String) {
        val db = Firebase.firestore
        val messageData = hashMapOf(
            "text" to text,
            "sender" to "user",
            "timestamp" to com.google.firebase.Timestamp.now(),
            "status" to "pending"
        )
        db.collection("Messages").add(messageData)
            .addOnSuccessListener {
                println("Message added to Firestore: $text")
            }
            .addOnFailureListener { e ->
                println("Failed to add message: ${e.message}")
            }
    }

    // Load saved messages from SharedPreferences
    LaunchedEffect(Unit) {
        val prefs = context.getSharedPreferences("ChatLogs", Context.MODE_PRIVATE)
        val savedLogs = prefs.getStringSet("logs", emptySet())?.toList() ?: emptyList()
        savedLogs.forEach { log ->
            val parts = log.split(":", limit = 3)
            if (parts.size == 3) {
                messages.add(Triple(parts[1].trim(), parts[0].trim(), parts[2].trim()))
            }
        }
    }

    // Save messages to SharedPreferences
    fun saveMessages() {
        val prefs = context.getSharedPreferences("ChatLogs", Context.MODE_PRIVATE)
        val logs = messages.map { "${it.second}:${it.first}:${it.third}" }.toSet()
        prefs.edit { putStringSet("logs", logs) }
    }

    // Clear messages from Firestore and SharedPreferences
    fun clearMessages() {
        messages.clear()
        val db = Firebase.firestore
        db.collection("Messages").get().addOnSuccessListener { documents ->
            for (document in documents) {
                document.reference.delete()
            }
        }
        val prefs = context.getSharedPreferences("ChatLogs", Context.MODE_PRIVATE)
        prefs.edit { remove("logs") }
    }

    // Dialog for confirming clear action
    if (showClearDialog) {
        AlertDialog(
            onDismissRequest = { showClearDialog = false },
            title = { Text("Clear Logs") },
            text = { Text("Do you want to delete all chat logs?") },
            confirmButton = {
                Button(onClick = {
                    clearMessages()
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

    // Chat UI layout
    Column(modifier = modifier.padding(16.dp)) {
        // Header
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.Start
        ) {
            Text(
                text = "Robot Command Chat",
                style = MaterialTheme.typography.headlineMedium,
                color = MaterialTheme.colorScheme.onBackground
            )
        }
        HorizontalDivider(
            modifier = Modifier
                .fillMaxWidth()
                .padding(vertical = 8.dp),
            thickness = 1.dp,
            color = Color.Gray
        )
        Spacer(modifier = Modifier.height(16.dp))

        // Message list
        LazyColumn(
            modifier = Modifier.weight(1f).padding(bottom = 16.dp),
            reverseLayout = true
        ) {
            items(messages.size) { index ->
                val reversedIndex = messages.size - 1 - index
                val (text, sender, timestamp) = messages[reversedIndex]
                MessageBubble(
                    text = text,
                    isUser = sender == "user",
                    timestamp = timestamp,
                    showIcon = sender != "user"
                )
            }
        }

        // Command buttons
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(bottom = 8.dp),
            horizontalArrangement = Arrangement.SpaceEvenly
        ) {
            CommandButton("Return", messages, ::sendMessage) { saveMessages() }
            CommandButton("Standby", messages, ::sendMessage) { saveMessages() }
            CommandButton("Resume", messages, ::sendMessage) { saveMessages() }
            ClearLogButton { showClearDialog = true }
        }

        HorizontalDivider(
            modifier = Modifier
                .fillMaxWidth()
                .padding(vertical = 8.dp),
            thickness = 1.dp,
            color = Color.Gray
        )

        // Message input
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .background(
                    MaterialTheme.colorScheme.surface,
                    shape = RoundedCornerShape(24.dp)
                )
                .padding(horizontal = 8.dp, vertical = 4.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            TextField(
                value = message,
                onValueChange = { message = it },
                modifier = Modifier
                    .weight(1f)
                    .background(Color.Transparent)
                    .border(0.5.dp, Color.White, RoundedCornerShape(8.dp)),
                placeholder = { Text("Enter command...", color = Color.Gray) },
                colors = TextFieldDefaults.colors(
                    unfocusedContainerColor = Color.Transparent,
                    focusedContainerColor = Color.Transparent,
                    focusedIndicatorColor = Color.Transparent,
                    unfocusedIndicatorColor = Color.Transparent,
                    disabledIndicatorColor = Color.Transparent,
                    cursorColor = MaterialTheme.colorScheme.primary,
                    focusedTextColor = MaterialTheme.colorScheme.onSurface,
                    unfocusedTextColor = MaterialTheme.colorScheme.onSurface
                )
            )
            IconButton(
                onClick = {
                    if (message.isNotBlank()) {
                        sendMessage(message)
                        saveMessages()
                        message = ""
                    }
                }
            ) {
                Icon(
                    imageVector = Icons.AutoMirrored.Filled.Send,
                    contentDescription = "Send",
                    tint = MaterialTheme.colorScheme.primary
                )
            }
        }
    }
}

// Command button with click animation
@Composable
fun CommandButton(
    command: String,
    messages: MutableList<Triple<String, String, String>>,
    sendMessage: (String) -> Unit,
    onSave: () -> Unit
) {
    var scale by remember { mutableStateOf(1f) }
    Surface(
        modifier = Modifier
            .scale(scale)
            .clickable(
                onClick = {
                    scale = 0.95f
                    val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.getDefault())
                    val timestamp = sdf.format(java.util.Date())
                    messages.add(Triple(command, "user", timestamp))
                    sendMessage(command)
                    onSave()
                }
            )
            .animateContentSize(animationSpec = spring()),
        shape = RectangleShape,
        color = Color(0xFF666666),
        tonalElevation = 4.dp
    ) {
        Text(
            text = command,
            modifier = Modifier.padding(horizontal = 16.dp, vertical = 8.dp),
            color = Color.White,
            style = MaterialTheme.typography.bodyMedium
        )
    }
}

// Clear log button with click animation
@Composable
fun ClearLogButton(onClick: () -> Unit) {
    var scale by remember { mutableStateOf(1f) }
    Surface(
        modifier = Modifier
            .scale(scale)
            .clickable(
                onClick = {
                    scale = 0.95f
                    onClick()
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

// Message bubble for displaying chat messages
@Composable
fun MessageBubble(text: String, isUser: Boolean, timestamp: String, showIcon: Boolean = false) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 8.dp),
        horizontalArrangement = if (isUser) Arrangement.End else Arrangement.Start
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically
        ) {
            if (showIcon) {
                Icon(
                    imageVector = Icons.Default.CheckCircle,
                    contentDescription = "Carry out task",
                    tint = MaterialTheme.colorScheme.primary,
                    modifier = Modifier.padding(end = 8.dp)
                )
            }
            Surface(
                shape = RoundedCornerShape(16.dp),
                color = if (isUser) MaterialTheme.colorScheme.primary else Color.Black,
                tonalElevation = 2.dp,
                modifier = if (isUser) {
                    Modifier.border(
                        width = 1.5.dp,
                        color = Color(0xFF1E90FF),
                        shape = RoundedCornerShape(16.dp)
                    )
                        .widthIn(max = 170.dp)
                } else {
                    Modifier.border(
                        width = 0.3.dp,
                        color = Color.White,
                        shape = RoundedCornerShape(16.dp)
                    )
                        .widthIn(max = 200.dp)
                }
            ) {
                Column(
                    modifier = Modifier.padding(12.dp)
                ) {
                    Text(
                        text = text,
                        color = if (isUser) MaterialTheme.colorScheme.onPrimary else Color.White,
                        style = MaterialTheme.typography.bodySmall
                    )
                    Text(
                        text = timestamp,
                        color = Color.Gray,
                        style = MaterialTheme.typography.bodySmall,
                        modifier = Modifier.padding(top = 4.dp)
                    )
                }
            }
        }
    }
}