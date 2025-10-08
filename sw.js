const CACHE_NAME = 'ros-control-pwa-v1.0.0';
const urlsToCache = [
  '/map_viewer.html',
  '/manifest.json',
  'https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
  'https://cdn.jsdelivr.net/npm/nipplejs@0.10.2/dist/nipplejs.min.js'
];

// Install Service Worker
self.addEventListener('install', function(event) {
  console.log('PWA: Service Worker installing...');
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(function(cache) {
        console.log('PWA: Opened cache');
        return cache.addAll(urlsToCache);
      })
      .then(function() {
        console.log('PWA: All resources cached');
        return self.skipWaiting(); // Activate immediately
      })
  );
});

// Activate Service Worker
self.addEventListener('activate', function(event) {
  console.log('PWA: Service Worker activating...');
  event.waitUntil(
    caches.keys().then(function(cacheNames) {
      return Promise.all(
        cacheNames.map(function(cacheName) {
          if (cacheName !== CACHE_NAME) {
            console.log('PWA: Deleting old cache:', cacheName);
            return caches.delete(cacheName);
          }
        })
      );
    }).then(function() {
      console.log('PWA: Service Worker activated');
      return self.clients.claim(); // Take control immediately
    })
  );
});

// Fetch with Cache Strategy
self.addEventListener('fetch', function(event) {
  // Skip cross-origin requests and non-GET requests
  if (!event.request.url.startsWith(self.location.origin) && 
      !event.request.url.startsWith('https://cdn.jsdelivr.net') &&
      !event.request.url.startsWith('https://unpkg.com')) {
    return;
  }
  
  if (event.request.method !== 'GET') {
    return;
  }

  event.respondWith(
    caches.match(event.request)
      .then(function(response) {
        // Return cached version if available
        if (response) {
          console.log('PWA: Serving from cache:', event.request.url);
          return response;
        }

        // Clone the request because it's a stream
        const fetchRequest = event.request.clone();

        return fetch(fetchRequest).then(function(response) {
          // Check if we received a valid response
          if (!response || response.status !== 200 || response.type !== 'basic') {
            return response;
          }

          // Clone the response because it's a stream
          const responseToCache = response.clone();

          caches.open(CACHE_NAME)
            .then(function(cache) {
              console.log('PWA: Caching new resource:', event.request.url);
              cache.put(event.request, responseToCache);
            });

          return response;
        }).catch(function(error) {
          console.log('PWA: Fetch failed, serving from cache if available:', error);
          
          // Try to serve a cached version for navigation requests
          if (event.request.mode === 'navigate') {
            return caches.match('/map_viewer.html');
          }
          
          throw error;
        });
      })
  );
});

// Background Sync for offline actions
self.addEventListener('sync', function(event) {
  console.log('PWA: Background sync triggered:', event.tag);
  
  if (event.tag === 'ros-command-sync') {
    event.waitUntil(
      // Handle offline ROS commands when connection is restored
      handleOfflineRosCommands()
    );
  }
});

// Push Notifications (for future robot status updates)
self.addEventListener('push', function(event) {
  console.log('PWA: Push message received');
  
  const options = {
    body: event.data ? event.data.text() : 'Robot status update',
    icon: 'data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 192 192"><circle cx="96" cy="96" r="80" fill="%23333"/><text y="120" x="96" text-anchor="middle" fill="white" font-size="60">🤖</text></svg>',
    badge: 'data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 96 96"><circle cx="48" cy="48" r="40" fill="%23333"/><text y="60" x="48" text-anchor="middle" fill="white" font-size="30">🤖</text></svg>',
    vibrate: [100, 50, 100],
    tag: 'ros-notification',
    renotify: true
  };

  event.waitUntil(
    self.registration.showNotification('ROS Robot Control', options)
  );
});

// Handle offline ROS commands
async function handleOfflineRosCommands() {
  // This would handle any queued ROS commands when connection is restored
  console.log('PWA: Handling offline ROS commands...');
  // Implementation would depend on your specific offline strategy
}

// Message handling from main thread
self.addEventListener('message', function(event) {
  console.log('PWA: Message received:', event.data);
  
  if (event.data && event.data.type === 'SKIP_WAITING') {
    self.skipWaiting();
  }
});
