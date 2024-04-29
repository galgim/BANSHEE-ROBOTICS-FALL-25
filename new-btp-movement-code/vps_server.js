import React, { useEffect, useRef } from 'react';
import '../styles/VideoFeed.css';

function VideoFeed() {

    // reference for the video element
    const videoRef = useRef(null);

    // reference for the WebSocket connection
    const wsRef = useRef(null);

    useEffect(() => {
        // establish WebSocket connection when component mounts to vps
        wsRef.current = new WebSocket("wss://rgs.bansheeuav.tech:3000/read_frames" );

        // an event listener for when the WebSocket connection is open
        wsRef.current.addEventListener('open', () => {
            console.log('Connected to WebSocket server for reading frames');
        });

        // an event listener for when a message is received through the WebSocket
        wsRef.current.addEventListener('message', (event) => {
            // retrieve the data from the WebSocket message
            const arrayBuffer = event.data;
            // convert the data into a Blob
            const blob = new Blob([arrayBuffer], { type: 'image/jpeg' });
            // create a URL for the Blob object
            const imageUrl = URL.createObjectURL(blob);
            // set the source of the video element to the created URL
            videoRef.current.src = imageUrl;
        });

        return () => {
            // close WebSocket connection when component unmounts
            if (wsRef.current) {
                wsRef.current.close();
            }
        };
    }, []); // The empty dependency array ensures that this effect runs only once

    // Render the component
    return (
        <div className='live-vid-container'>
            <h1>Live Video Stream</h1>
            <div>
                {/* Render an image element with a reference to the videoRef */}
                <img ref={videoRef} alt='vid'/>
            </div>
        </div>
    );
}

export default VideoFeed;