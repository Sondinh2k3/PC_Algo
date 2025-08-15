from mysql import connector

class SqlCollector:
    def __init__(self, host: str, port: int, user: str, password: str, database: str):
        self.connection_config = {
            "host": host,
            "port": port,
            "user": user,
            "password": password,
            "database": database,
            "autocommit": False,
            "pool_size": 5,
            "pool_reset_session": True
        }
        self.conn = None
        self.connection_attempts = 0
        self.max_connection_attempts = 3
        self._connect()

    def _connect(self):
        """Establish database connection with retry logic"""
        self.connection_attempts += 1
        try:
            if self.conn:
                try:
                    self.conn.close()
                except:
                    pass  # Ignore errors when closing
            self.conn = connector.connect(**self.connection_config)
            print("[INFO] MySQL connection established successfully")
            self.connection_attempts = 0  # Reset on successful connection
        except connector.Error as e:
            print(f"[ERROR] Failed to connect to MySQL (attempt {self.connection_attempts}): {e}")
            self.conn = None
            if self.connection_attempts >= self.max_connection_attempts:
                print(f"[ERROR] Max connection attempts reached. Database operations will be skipped.")

    def get_lane_area_detector_ids(self) -> list:
        """Retrieve all lane area detector IDs from the database"""
        detector_ids = []
        
        if not self.conn:
            print("[ERROR] No database connection available")
            return detector_ids
            
        try:
            cursor = self.conn.cursor()
            query = "SELECT id FROM lane_area_detector ORDER BY id"
            cursor.execute(query)
            
            results = cursor.fetchall()
            detector_ids = [row[0] for row in results]
            
            print(f"[INFO] Retrieved {len(detector_ids)} lane area detector IDs from database")
            cursor.close()
            
        except connector.Error as e:
            print(f"[ERROR] Failed to retrieve lane area detector IDs: {e}")
            # Try to reconnect if connection was lost
            if not self.conn.is_connected():
                self._connect()
                
        return detector_ids
    
    def reset_connection_attempts(self):
        """Reset connection attempts counter (useful for long-running processes)"""
        self.connection_attempts = 0

    def close(self):
        """Close database connection"""
        if self.conn and self.conn.is_connected():
            self.conn.close()
            print("[INFO] MySQL connection closed")