## Insights Gained

1. If there's a main thread and a background thread that need to communicate, i.e. the background thread may need to receive data from the main thread for tasks such as writing to disk, logging, or displaying monitoring information. In this case, we can consider whether the communication data can be divided into "metadata part and variable part". The metadata part is sent only once during compilation or initialization, and only the variable part needs to be sent in each subsequent communication.

2. If there are multiple main threads, we certainly do not want the communication process to cause significant delays to the main threads. Therefore, we can consider establishing a SPSC (Single Producer Single Consumer) Queue for each communication between a main thread and the background thread, rather than creating an MPSC (Multiple Producer Single Consumer) Queue. This approach minimizes the impact on the main threads.

3. Strive to complete as much computation as possible at compile time, such as the "CStringSizes" part in the code.