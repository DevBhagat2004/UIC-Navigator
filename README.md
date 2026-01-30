# UIC Navigator

An intelligent route-finding web application designed specifically for University of Illinois Chicago (UIC) students to discover the shortest paths between campus buildings and classes.

## Overview

UIC Navigator helps students save time on campus by finding the shortest routes between buildings. It uses real map data and a reliable pathfinding algorithm to give you accurate directions across the UIC campus.

## Features

- **Shortest Path Calculation**: Utilizes Dijkstra's algorithm to find optimal routes between campus locations
- **Real Campus Data**: Integrated with UIC campus building information for accurate location mapping
- **OpenStreetMap Integration**: Uses real street and pathway data from OpenStreetMap (OSM) for precise routing
- **Interactive Web Interface**: User-friendly web application accessible through your browser
- **Local Deployment**: Runs on localhost for fast, reliable performance

## Technology Stack

- **Backend**: C++ (for high-performance pathfinding computations)
- **Frontend**: JavaScript (for interactive user interface)
- **Map Data**: OpenStreetMap (OSM)
- **Algorithm**: Dijkstra's shortest path algorithm

## Prerequisites

Before running UIC Navigator, ensure you have the following installed:

- C++ compiler (GCC 7+ or Clang 5+)
- Make (for building the project)
- Modern web browser (Chrome, Firefox, Safari, or Edge)

## Installation

1. Clone the repository:
```bash
git clone https://github.com/DevBhagat2004/UIC-Navigator.git
cd UIC-Navigator
```

2. Build and run the server:
```bash
make run_server
```

3. Open your web browser and navigate to:
```
http://localhost:1251
```

4. To stop the server, press `CTRL-C` in the terminal

## Usage

1. **Select Starting Location**: Choose your current building or location on the UIC campus
2. **Select Destination**: Pick your destination building or classroom
3. **View Route**: The application will display the shortest path between your locations
4. **Follow Directions**: Use the generated route to navigate across campus

## How It Works

1. **Data Loading**: The application loads UIC campus building data and OpenStreetMap street information
2. **Graph Construction**: Creates a graph representation of the campus with buildings as nodes and pathways as edges
3. **Route Calculation**: Implements Dijkstra's algorithm to compute the shortest path between selected locations
4. **Visualization**: Displays the calculated route on an interactive map interface

## Algorithm

UIC Navigator uses **Dijkstra's algorithm**, a proven graph search algorithm that guarantees finding the shortest path between nodes in a weighted graph. This ensures students always get the most efficient route to their destinations.

## Data Sources

- **Campus Buildings**: Custom dataset of UIC building locations and information
- **Street Data**: OpenStreetMap (OSM) for accurate pathway and street mapping
