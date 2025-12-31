#include <cstdint>
#include <memory>
#include <cassert>
#include <algorithm>

class FlowSolver {
    struct color_features_t {
        int color_index;
        int wall_dist[2];
        int min_dist;
    };

    using pos_t = uint8_t;
    using cell_t = uint8_t;

    static constexpr pos_t INVALID_POS = 0xff;
    static constexpr int MAX_COLORS = 16;
    static constexpr int MAX_SIZE = 15;
    static constexpr int MAX_CELLS = (MAX_SIZE + 1) * MAX_SIZE - 1;

    enum CELL_TYPE {FREE = 0, PATH = 1, INIT = 2, GOAL = 3};
    enum DIRECTION {LEFT = 0, RIGHT = 1, UP = 2, DOWN = 3};
    enum SEARCH {SUCCESS = 0, UNREACHABLE = 1, FULL = 2, IN_PROGRESS = 3};

    const int DIR_DELTA[4][3] = {
        { -1, 0, -1 },
        {  1, 0,  1 },
        {  0, -1, -16 },
        {  0, 1, 16 }
    };

    // init/game info
    cell_t cells[MAX_CELLS]{0};
    pos_t init_pos[MAX_COLORS];
    pos_t goal_pos[MAX_COLORS];

    size_t size;
    size_t num_colors;
    
    // game state
    cell_t   cells[MAX_CELLS];
    pos_t    pos[MAX_COLORS];
    uint8_t  num_free;
    uint8_t  last_color;
    uint16_t completed;
    int color_order[MAX_COLORS];

    FlowSolver (size_t size, size_t num_colors): size(size), num_colors(num_colors), num_free(0), last_color(-1), completed(0) {
        memset(init_pos, 0xff, sizeof(init_pos));
        memset(goal_pos, 0xff, sizeof(goal_pos));
    }
    // create cells
    cell_t create_cell(uint8_t type, uint8_t color, uint8_t dir) {
        return (color & 0xf) << 4 | (dir & 0x3) << 2 | (dir & 0x3) << 2;
    }

    // cell getters
    uint8_t cell_get_color(cell_t cell) {
        return (cell >> 4) & 0xf;
    }

    uint8_t cell_get_type(cell_t cell) {
        return (cell >> 2) & 0x3;
    }

    uint8_t cell_get_dir(cell_t cell) {
        return cell & 0x3;
    }

    // pos_t to int conversion and vice versa
    pos_t pos_from_coords(int x, int y) {
        return (y & 0xf) << 4 | (x & 0xf);
    }

    void pos_to_coords(int* x, int* y, pos_t yx) {
        *x = (yx & 0xf); 
        *y = ((yx >> 4) & 0xf);
    }

    // checks for move validation
    bool coords_valid(int x, int y) {
        return x < size && x >= 0 && y < size && y >= 0;
    }

    bool pos_valid(pos_t yx) {
        return (yx & 0xf) < size && (yx & 0xf) >= 0 && ((yx >> 4) & 0xf) < size && ((yx >> 4) & 0xf) >= 0;
    }

    pos_t offset_pos (int x, int y, int dir) {
        int offset_x = x + DIR_DELTA[dir][0];
        int offset_y = y + DIR_DELTA[dir][1];

        return coords_valid(offset_x, offset_y) ? (offset_y & 0xf) << 4 | offset_x  & 0xf : INVALID_POS;
    }

    // minimum distance from a wall
    int get_wall_dist(int x, int y) {
        int d[2]{0};

        d[0] = x > size - x ? size - x : x;
        d[1] = y > size - y ? size - y : y;
        
        return x < y ? x : y;
    }

    int pos_get_wall_dist(pos_t yx) {
        int x, y;
        pos_to_coords(&x, &y, yx);
        return get_wall_dist(x, y);
    }

    // SOLVER LOGIC //////////////////////////////

    int coord_free_space(int x, int y) {
        int free{0};
        for (int dir{0}; dir < 4; dir++) {
            pos_t new_pos = offset_pos(x, y, dir);
            if (new_pos != INVALID_POS && cells[new_pos] == 0) free++;
        }
    }

    int pos_free_space (pos_t yx) {
        int x, y;
        pos_to_coords(&x, &y, yx);
        return coord_free_space(x, y);
    }

    // track by color
    int game_can_move(int color, int dir) {
        int cur_x, cur_y;
        pos_to_coords(&cur_x, &cur_y, pos[color]);

        pos_t new_pos = offset_pos(cur_x, cur_y, dir);

        if (!pos_valid(new_pos)) return 0;
        if (new_pos == goal_pos[color]) return 1;

        // checks if cell != 0 --> not free
        if (cells[new_pos]) {
            return 0;
        }

        // check if it's touching itself
        for (int i{0}; i < 4; i++) {
            pos_t neighbor = offset_pos(cur_x, cur_y, i);

            if (neighbor != INVALID_POS && 
                cells[neighbor] && 
                neighbor != cells[color] && 
                neighbor != goal_pos[color] && 
                cell_get_color(cells[neighbor]) != color) 
            return 0;
        }

        return 1;
    }

    int game_make_move(int color, int dir, int forced) {
        cell_t move = create_cell(PATH, color, dir);

        pos_t cur_pos = pos[color];

        int x, y;
        pos_to_coords(&x, &y, cur_pos);        
        pos_t new_pos = offset_pos(x, y, dir);

        assert(pos_valid(new_pos) && new_pos < MAX_CELLS);

        // check if new pos is the goal
        if (new_pos == goal_pos[color]) {
            cells[new_pos] = create_cell(GOAL, color, dir);
            completed |= 1 << color;
            return 0;
        }

        // update game state
        assert(cells[new_pos] == 0);
        cells[new_pos] = move;
        pos[color] = new_pos;
        last_color = color;

        // compute move action
        double action_cost{1};
        int goal_dir{-1};

        // check surrounding for goal
        for (int dir{0}; dir < 4; dir++) {
            if (offset_pos(x, y, dir) == goal_pos[color]) { 
                goal_dir = dir;
                break;
            }
        }

        if (goal_dir >= 0) {
            cells[goal_pos[color]] = create_cell(GOAL, color, goal_dir);
            completed |= (1 << color);
            action_cost = 0;
        } else {
            int free = coord_free_space(x, y);
            if (num_free == 2) action_cost = 2;
        }
        
        // action cost is 0;
        if (forced) return 0;

        return action_cost;
    }

    int next_color_to_move () {
        // continue last color or choose another non-deterministically
        if (!(completed & (1 < last_color))) return last_color;

        size_t best_color{-1};
        // int best_free{4};
        // int num_free{-1};

        // for (size_t color{0}; color < num_colors; color++) {
        //     int color_pos = pos[color];

        //     if (completed & (1 << color)) continue;
            
        //     num_free = pos_free_space(color_pos);
        //     if (num_free < best_free) {
        //         best_free = num_free;
        //         best_color = color;
        //     }
        // }

        for (size_t i{0}; i < num_colors; i++) {
            int color = color_order[i];
            if (completed & (1 << color)) { continue; }
            return color;
        }

        return best_color;
    }

    // SORTING COLORS ////////////////////////////////////////////////////////
    void order_colors () {
        color_features_t cf[MAX_COLORS]{0};
        for (size_t color{0}; color < num_colors; color++) {
            cf[color].color_index = color;

            int x[2], y[2];
            for (int i{0}; i < 2; i++) {
                pos_to_coords(x + i, y + i, pos[color]);
                cf[color].wall_dist[i] = get_wall_dist(x[i], y[i]);
            }

            int dx = abs(x[1] - x[0]);
            int dy = abs(y[1] - y[0]);
        
            cf[color].min_dist = dx + dy;
        }

        std::sort(cf, cf + num_colors, [](const color_features_t& a, const color_features_t& b) {
            if (a.wall_dist[0] != b.wall_dist[0]) return a.wall_dist[0] < b.wall_dist[0];
            if (a.wall_dist[1] != b.wall_dist[1]) return a.wall_dist[1] < b.wall_dist[1];
            return a.min_dist < b.min_dist;
        });

        for (size_t i{0}; i < num_colors; i++) {
            color_order[i] = cf[i].color_index;
        }
    }

};


int main () {
    return 0;
}