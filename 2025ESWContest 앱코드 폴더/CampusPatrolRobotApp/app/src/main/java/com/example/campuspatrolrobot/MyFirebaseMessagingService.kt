package com.example.campuspatrolrobot

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Intent
import androidx.core.app.NotificationCompat
import com.google.firebase.messaging.FirebaseMessagingService
import com.google.firebase.messaging.RemoteMessage
import java.util.concurrent.atomic.AtomicInteger

class MyFirebaseMessagingService : FirebaseMessagingService() {

    companion object {
        private const val GROUP_KEY_ROBOT = "com.example.campuspatrolrobot.ROBOT_NOTIFICATIONS"
        private val notificationIdCounter = AtomicInteger(0)
    }

    private var lastKickboardCoords: Pair<Double, Double>? = null
    private var lastKickboardTime: Long = 0

    override fun onCreate() {
        super.onCreate()
        createNotificationChannels()
    }

    override fun onNewToken(token: String) {
        super.onNewToken(token)
    }

    // Handles incoming FCM messages and processes notifications
    override fun onMessageReceived(remoteMessage: RemoteMessage) {
        val state = remoteMessage.data["state"] ?: "unknown"
        val title = remoteMessage.data["title"] ?: "New notification"
        val body = remoteMessage.data["body"] ?: "No content"
        val eventType = remoteMessage.data["eventType"] ?: ""
        val latitude = remoteMessage.data["latitude"]?.toDoubleOrNull() ?: 0.0
        val longitude = remoteMessage.data["longitude"]?.toDoubleOrNull() ?: 0.0

        if (state !in listOf("01", "10", "11")) {
            return
        }

        if (state == "10") {
            val currentCoords = Pair(latitude, longitude)
            val currentTime = System.currentTimeMillis()
            if (lastKickboardCoords == currentCoords && (currentTime - lastKickboardTime) < 3000) {
                return
            }
            lastKickboardCoords = currentCoords
            lastKickboardTime = currentTime
        }

        val channelId = when (state) {
            "01" -> "patrol_complete_en"
            "10" -> "kickboard_detected_en"
            "11" -> "problem_detected_en"
            else -> "robot_state_en"
        }

        val iconRes = when (state) {
            "01" -> R.drawable.ic_patrol_complete
            "10" -> R.drawable.ic_kickboard
            "11" -> R.drawable.ic_problem
            else -> R.drawable.ic_notification
        }

        val colorRes = when (state) {
            "01" -> 0xFF4CAF50.toInt()
            "10" -> 0xFF2196F3.toInt()
            "11" -> 0xFFF44336.toInt()
            else -> 0xFF9E9E9E.toInt()
        }

        val intent = Intent(this, MainActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP
            putExtra("state", state)
            putExtra("eventType", eventType)
        }
        val pendingIntent = PendingIntent.getActivity(
            this,
            notificationIdCounter.getAndIncrement(),
            intent,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )

        val notificationId = notificationIdCounter.getAndIncrement()
        val notificationManager = getSystemService(NotificationManager::class.java)

        val builder = NotificationCompat.Builder(this, channelId)
            .setSmallIcon(iconRes)
            .setContentTitle(title)
            .setContentText(body)
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setAutoCancel(true)
            .setContentIntent(pendingIntent)
            .setColor(colorRes)
            .setGroup(GROUP_KEY_ROBOT)

        notificationManager.notify(notificationId, builder.build())

        // Creates summary notification for grouped notifications
        val summaryId = GROUP_KEY_ROBOT.hashCode()
        val summaryBuilder = NotificationCompat.Builder(this, channelId)
            .setSmallIcon(R.drawable.ic_notification)
            .setContentTitle("Robot notifications")
            .setContentText("New notifications have arrived.")
            .setGroup(GROUP_KEY_ROBOT)
            .setGroupSummary(true)
            .setAutoCancel(true)
        notificationManager.notify(summaryId, summaryBuilder.build())
    }

    // Initializes notification channels for Android O and above
    private fun createNotificationChannels() {
        val channels = listOf(
            NotificationChannel(
                "robot_state_en",
                "Robot status notifications",
                NotificationManager.IMPORTANCE_HIGH
            ).apply {
                enableVibration(true)
                setBypassDnd(true)
            },
            NotificationChannel(
                "patrol_complete_en",
                "Patrol completed notifications",
                NotificationManager.IMPORTANCE_HIGH
            ).apply {
                enableVibration(true)
                setBypassDnd(true)
            },
            NotificationChannel(
                "kickboard_detected_en",
                "Kickboard detected notifications",
                NotificationManager.IMPORTANCE_HIGH
            ).apply {
                enableVibration(true)
                setBypassDnd(true)
            },
            NotificationChannel(
                "problem_detected_en",
                "Problem detected notifications",
                NotificationManager.IMPORTANCE_HIGH
            ).apply {
                enableVibration(true)
                setBypassDnd(true)
            }
        )

        val manager = getSystemService(NotificationManager::class.java)
        manager.createNotificationChannels(channels)
    }
}