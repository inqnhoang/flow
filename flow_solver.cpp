#include <cstdint>
#include <memory>
#include <cassert>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <queue>
#include <fstream>
#include <iostream>

class FlowSolver {
    using pos_t = uint8_t;
    using cell_t = uint8_t;

    static constexpr pos_t INVALID_POS = 0xff;
    static constexpr int MAX_COLORS = 16;
    static constexpr int MAX_SIZE = 15;
    static constexpr int MAX_CELLS = (MAX_SIZE + 1) * MAX_SIZE - 1;

    struct game_state_t {
        cell_t   cells[MAX_CELLS];
        pos_t    pos[MAX_COLORS];
        uint8_t  num_free;
        int8_t   last_color;
        uint16_t completed;
        uint8_t colors_unsolved;

        game_state_t(): cells{0}, pos{0}, num_free{0}, last_color{-1}, completed{0} {};
    };

    struct color_features_t {
        int color_index;
        int wall_dist[2];
        int min_dist;
    };

    struct region_t {
        pos_t parent;
        uint8_t rank;
    };

    struct tree_node_t {
        game_state_t game_state;
        double cost_to_come;
        double cost_to_go;
        tree_node_t* parent;
    };

    struct node_storage_t {
        tree_node_t* start;
        size_t capacity;
        size_t count; 
    };

    struct node_compare {
        bool operator()(const tree_node_t* a, const tree_node_t* b) const {
            double af = a->cost_to_come + a->cost_to_go;
            double bf = b->cost_to_come + b->cost_to_go;
            
            if (af != bf) return af > bf;  // Reversed for min-heap!
            return a > b;
        }
    };

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
    pos_t init_pos[MAX_COLORS];
    pos_t goal_pos[MAX_COLORS];

    size_t size;
    size_t num_colors;
    
    // game state
    int color_order[MAX_COLORS];




public:
    game_state_t* solution;

    FlowSolver() : size(0), num_colors(0), solution(nullptr) {
        memset(init_pos, 0xff, sizeof(init_pos));
        memset(goal_pos, 0xff, sizeof(goal_pos));
        memset(color_order, 0, sizeof(color_order));

        printf("size: %zu\n", size);
    }

    // create cells - Fixed to match C version bit layout: [color:4][dir:2][type:2]
    cell_t create_cell (uint8_t type, uint8_t color, uint8_t dir) {
        return ((color & 0xf) << 4) | ((dir & 0x3) << 2) | (type & 0x3);
    }

    // cell getters - Fixed to match bit layout
    uint8_t cell_get_color (cell_t cell) {
        return (cell >> 4) & 0xf;
    }

    uint8_t cell_get_type (cell_t cell) {
        return cell & 0x3;
    }

    uint8_t cell_get_dir (cell_t cell) {
        return (cell >> 2) & 0x3;
    }

    // pos_t to int conversion and vice versa
    pos_t pos_from_coords (int x, int y) {
        return (y & 0xf) << 4 | (x & 0xf);
    }

    void pos_to_coords (int* x, int* y, pos_t yx) {
        *x = (yx & 0xf); 
        *y = ((yx >> 4) & 0xf);
    }

    // checks for move validation
    bool coords_valid (int x, int y) {
        return x < (int)size && x >= 0 && y < (int)size && y >= 0;
    }

    bool pos_valid (pos_t yx) {
        // printf("x: %d y: %d", yx & 0xf, (yx >> 4) & 0xf);
        return (yx & 0xf) < size && ((yx >> 4) & 0xf) < size;
    }

    pos_t offset_pos (int x, int y, int dir) {
        int offset_x = x + DIR_DELTA[dir][0];
        int offset_y = y + DIR_DELTA[dir][1];

        return coords_valid(offset_x, offset_y) ? 
            pos_from_coords(offset_x, offset_y) : INVALID_POS;
    }

    // minimum distance from a wall
    int get_wall_dist (int x, int y) {
        int d[2];
        d[0] = std::min(x, (int)size - 1 - x);
        d[1] = std::min(y, (int)size - 1 - y);
        return std::min(d[0], d[1]);
    }

    int pos_get_wall_dist(pos_t yx) {
        int x, y;
        pos_to_coords(&x, &y, yx);
        return get_wall_dist(x, y);
    }

    // SOLVER LOGIC //////////////////////////////

    int coord_free_space (game_state_t* state, int x, int y) {
        int free = 0;
        for (int dir = 0; dir < 4; dir++) {
            pos_t new_pos = offset_pos(x, y, dir);
            if (new_pos != INVALID_POS && state->cells[new_pos] == 0) free++;
        }
        return free;
    }

    int pos_free_space(game_state_t* state, pos_t yx) {
        int x, y;
        pos_to_coords(&x, &y, yx);
        return coord_free_space(state, x, y);
    }

    int game_can_move (game_state_t* state, int color, int dir) {
        int cur_x, cur_y;
        pos_to_coords(&cur_x, &cur_y, state->pos[color]);

        pos_t new_pos = offset_pos(cur_x, cur_y, dir);

        if (!pos_valid(new_pos)) return 0;
        if (new_pos == goal_pos[color] && state->colors_unsolved > 1) return 1;
        else if (state->colors_unsolved == 1 && state->num_free == 0) return 1;

        if (state->cells[new_pos]) {
            printf("occupado brocacho\n");
            return 0;
        }

        int new_x, new_y;
        pos_to_coords(&new_x, &new_y, new_pos);
        
        for (int i = 0; i < 4; i++) {
            pos_t neighbor = offset_pos(new_x, new_y, i);
            if (neighbor == INVALID_POS || neighbor >= MAX_CELLS) continue;

            if (neighbor == INVALID_POS && 
                state->cells[neighbor] && 
                neighbor != state->pos[color] && 
                neighbor != goal_pos[color] && 
                cell_get_color(state->cells[neighbor]) == color) {
                return 0;
            }
        }

        return 1;
    }

    double game_make_move (game_state_t* state, int color, int dir, bool forced) {
        cell_t move = create_cell(PATH, color, dir);

        pos_t cur_pos = state->pos[color];

        int x, y;
        pos_to_coords(&x, &y, cur_pos);        
        pos_t new_pos = offset_pos(x, y, dir);

        assert(pos_valid(new_pos) && new_pos < MAX_CELLS);

        if (new_pos == goal_pos[color]) {
            state->cells[new_pos] = create_cell(GOAL, color, dir);
            state->completed |= 1 << color;
            --state->colors_unsolved;
            return 0;
        }

        if (state->cells[new_pos] != 0) return 2;
        state->cells[new_pos] = move;
        state->pos[color] = new_pos;
        state->last_color = color;
        state->num_free--;

        double action_cost = 1;
        int goal_dir = -1;

        int new_x, new_y;
        pos_to_coords(&new_x, &new_y, new_pos);

        for (int d = 0; d < 4; d++) {
            if (offset_pos(new_x, new_y, d) == goal_pos[color]) { 
                goal_dir = d;
                break;
            }
        }

        if (goal_dir >= 0) {
            if (state->colors_unsolved > 1) {
                state->cells[goal_pos[color]] = create_cell(GOAL, color, goal_dir);
                state->completed |= (1 << color);
                action_cost = 0;
                --state->colors_unsolved;
            }
        } else {
            int free = coord_free_space(state, new_x, new_y);
            if (free == 1) action_cost = 2;
        }
        
        if (forced) return 0;

        return action_cost;
    }

    int next_color_to_move (game_state_t* state) {
        if (state->last_color >= 0 && !(state->completed & (1 << state->last_color))) {
            return state->last_color;
        }

        for (size_t i = 0; i < num_colors; i++) {
            int color = color_order[i];
            if (!(state->completed & (1 << color))) {
                return color;
            }
        }

        return -1;
    }

    // SORTING COLORS ////////////////////////////////////////////////////////
    void order_colors () {
        color_features_t cf[MAX_COLORS];
        
        for (size_t color = 0; color < num_colors; color++) {
            cf[color].color_index = color;

            int x[2], y[2];
            pos_to_coords(&x[0], &y[0], init_pos[color]);
            pos_to_coords(&x[1], &y[1], goal_pos[color]);
            
            cf[color].wall_dist[0] = get_wall_dist(x[0], y[0]);
            cf[color].wall_dist[1] = get_wall_dist(x[1], y[1]);

            int dx = abs(x[1] - x[0]);
            int dy = abs(y[1] - y[0]);
        
            cf[color].min_dist = dx + dy;
        }

        std::sort(cf, cf + num_colors, [](const color_features_t& a, const color_features_t& b) {
            if (a.wall_dist[0] != b.wall_dist[0]) return a.wall_dist[0] < b.wall_dist[0];
            if (a.wall_dist[1] != b.wall_dist[1]) return a.wall_dist[1] < b.wall_dist[1];
            return a.min_dist < b.min_dist;
        });

        for (size_t i = 0; i < num_colors; i++) {
            color_order[i] = cf[i].color_index;
        }
    }

    // REGIONS ////////////////////////////////
    region_t create_region (pos_t yx) {
        return region_t{yx, 0};
    }

    pos_t find_region (region_t* regions, pos_t target) {
        if (regions[target].parent != target) {
            regions[target].parent = find_region(regions, regions[target].parent);
        }
        return regions[target].parent;
    }

    void unite_region (region_t* regions, pos_t a, pos_t b) {
        pos_t root_a = find_region(regions, a);
        pos_t root_b = find_region(regions, b);

        if (root_a == root_b) return;

        if (regions[root_a].rank < regions[root_b].rank) {
            regions[root_a].parent = root_b;
        } else if (regions[root_a].rank > regions[root_b].rank) {
            regions[root_b].parent = root_a;
        } else {
            regions[root_b].parent = root_a;
            ++regions[root_a].rank;
        }
    }

    size_t build_regions (game_state_t* state, uint8_t rmap[MAX_CELLS]) {
        region_t regions[MAX_CELLS];

        for (int y = 0; y < (int)size; y++) {
            for (int x = 0; x < (int)size; x++) {
                pos_t p = pos_from_coords(x, y);
                
                if (state->cells[p]) {
                    regions[p] = create_region(INVALID_POS);
                } else {
                    regions[p] = create_region(p);
                }

                if (x > 0) {
                    pos_t x_pos = pos_from_coords(x - 1, y);
                    if (!state->cells[x_pos] && !state->cells[p]) {
                        unite_region(regions, p, x_pos);
                    }
                }

                if (y > 0) {
                    pos_t y_pos = pos_from_coords(x, y - 1);
                    if (!state->cells[y_pos] && !state->cells[p]) {
                        unite_region(regions, p, y_pos);
                    }
                }
            } 
        }

        uint8_t rlookup[MAX_CELLS];
        size_t rcount = 0;

        memset(rlookup, 0xff, sizeof(rlookup));
        memset(rmap, 0xff, MAX_CELLS);

        for (int y = 0; y < (int)size; y++) {
            for (int x = 0; x < (int)size; x++) {
                pos_t p = pos_from_coords(x, y);
                
                if (state->cells[p]) {
                    rmap[p] = INVALID_POS;
                } else {
                    pos_t root = find_region(regions, p);
                    if (root != INVALID_POS) {
                        if (rlookup[root] == 0xff) {
                            rlookup[root] = rcount++;
                        }
                        rmap[p] = rlookup[root];
                    }
                }
            }
        }

        return rcount;
    }

    void regions_add_color (const uint8_t rmap[MAX_CELLS],
                          pos_t p,
                          uint16_t cflag,
                          uint16_t* rflags) {

        int x, y;
        pos_to_coords(&x, &y, p);
        
        for (int dir = 0; dir < 4; dir++) {
            pos_t neighbor_pos = offset_pos(x, y, dir);

            if (neighbor_pos != INVALID_POS) {
                uint8_t neighbor_region = rmap[neighbor_pos];

                if (neighbor_region != 0xff) {
                    rflags[neighbor_region] |= cflag;
                }
            }
        }
    }

    // PRUNING //////////////////

    bool game_is_deadend (game_state_t* state, pos_t p) {
        int x, y;
        pos_to_coords(&x, &y, p);
    
        int num_free = 0;
        for (int dir = 0; dir < 4; dir++) {
            pos_t neighbor_pos = offset_pos(x, y, dir);
            if (neighbor_pos != INVALID_POS) {
                if (!state->cells[neighbor_pos]) {
                    num_free++;
                } else {
                    for (size_t color = 0; color < num_colors; color++) {
                        if (state->completed & (1 << color)) continue;
                        
                        if (neighbor_pos == state->pos[color] ||
                            neighbor_pos == goal_pos[color]) {
                            num_free++;
                            break;
                        }
                    }
                }
            }
        }

        return num_free <= 1;
    }

    bool game_check_deadends (game_state_t* state) {
        pos_t curr_pos = state->pos[state->last_color];

        int x, y;
        pos_to_coords(&x, &y, curr_pos);
        
        for (int dir = 0; dir < 4; dir++) {
            pos_t neighbor_pos = offset_pos(x, y, dir);
            if (neighbor_pos != INVALID_POS &&
                !state->cells[neighbor_pos] &&
                game_is_deadend(state, neighbor_pos)) {
                return true;
            }
        }
        return false;
    }

    bool game_regions_stranded (game_state_t* state, size_t rcount,
                          const uint8_t rmap[MAX_CELLS],
                          size_t chokepoint_color,
                          int max_stranded) {
    
        uint16_t cur_rflags[MAX_CELLS] = {0};
        uint16_t goal_rflags[MAX_CELLS] = {0};
        
        int num_stranded = 0;
        
        for (size_t color = 0; color < num_colors; color++) {
            uint16_t cflag = (1 << color);
            
            if ((state->completed & cflag) || color == chokepoint_color) {
                continue;
            }
            
            regions_add_color(rmap, state->pos[color], cflag, cur_rflags);
            regions_add_color(rmap, goal_pos[color], cflag, goal_rflags);
            
            int delta = state->pos[color] - goal_pos[color];
            if (delta < 0) delta = -delta;
            if (delta == 1 || delta == 16) continue;
            
            bool can_reach = false;
            for (size_t r = 0; r < rcount; r++) {
                if ((cur_rflags[r] & cflag) && (goal_rflags[r] & cflag)) {
                    can_reach = true;
                    break;
                }
            }
            
            if (!can_reach) {
                num_stranded++;
                if (num_stranded > max_stranded) {
                    return true;
                }
            }
        }
        
        if (chokepoint_color >= num_colors) {
            for (size_t r = 0; r < rcount; r++) {
                if (!(cur_rflags[r] & goal_rflags[r])) {
                    return true;
                }
            }
        }
    
        return false;
    }

    bool is_forced_position (game_state_t* state, int color, pos_t p) {
        int x, y;
        pos_to_coords(&x, &y, p);
        
        int num_free = 0;
        int num_other_endpoints = 0;
        
        for (int dir = 0; dir < 4; dir++) {
            pos_t neighbor = offset_pos(x, y, dir);
            if (neighbor == INVALID_POS || neighbor == state->pos[color]) continue;
            
            if (!state->cells[neighbor]) {
                num_free++;
            } else {
                for (size_t c = 0; c < num_colors; c++) {
                    if (c == color || (state->completed & (1 << c))) continue;
                    if (neighbor == state->pos[c] || neighbor == goal_pos[c]) {
                        num_other_endpoints++;
                    }
                }
            }
        }
        
        return (num_free == 1 && num_other_endpoints == 0);
    }

    bool find_forced_move (game_state_t* state, int* forced_color, int* forced_dir) {
        for (size_t i = 0; i < num_colors; i++) {
            int color = color_order[i];
            if (state->completed & (1 << color)) continue;
            
            int cur_x, cur_y;
            pos_to_coords(&cur_x, &cur_y, state->pos[color]);
            
            for (int dir = 0; dir < 4; dir++) {
                pos_t neighbor = offset_pos(cur_x, cur_y, dir);
                if (neighbor == INVALID_POS) continue;
                
                if (!state->cells[neighbor] && is_forced_position(state, color, neighbor)) {
                    *forced_color = color;
                    *forced_dir = dir;
                    return true;
                }
            }
        }
        return false;
    }

    // ===== BOTTLENECK CHECKING =====

    bool is_free_cell (game_state_t* state, int x, int y) {
        return coords_valid(x, y) && state->cells[pos_from_coords(x, y)] == 0;
    }

    int check_chokepoint (game_state_t* state, int color, int dir, int n) {
        cell_t saved_cells[MAX_CELLS];
        pos_t saved_pos[MAX_COLORS];
        uint8_t saved_free = state->num_free;
        int8_t saved_last = state->last_color;
        uint16_t saved_completed = state->completed;
        
        memcpy(saved_cells, state->cells, sizeof(state->cells));
        memcpy(saved_pos, state->pos, sizeof(state->pos));
        
        for (int i = 0; i < n; i++) {
            if (!game_can_move(state, color, dir)) {
                memcpy(state->cells, saved_cells, sizeof(state->cells));
                memcpy(state->pos, saved_pos, sizeof(state->pos));
                state->num_free = saved_free;
                state->last_color = saved_last;
                state->completed = saved_completed;
                return 0;
            }
            game_make_move(state, color, dir, 1);
        }
        
        uint8_t rmap[MAX_CELLS];
        size_t rcount = build_regions(state, rmap);
        
        int result = 0;
        if (game_regions_stranded(state, rcount, rmap, color, n)) {
            result = 1;
        }
        
        memcpy(state->cells, saved_cells, sizeof(state->cells));
        memcpy(state->pos, saved_pos, sizeof(state->pos));
        state->num_free = saved_free;
        state->last_color = saved_last;
        state->completed = saved_completed;
        
        return result;
    }

    bool check_bottleneck (game_state_t* state, int bottleneck_limit = 3) {
        if (bottleneck_limit == 0) return false;
        if (state->last_color >= (int)num_colors) return false;
        
        pos_t p = state->pos[state->last_color];
        int x0, y0;
        pos_to_coords(&x0, &y0, p);
        
        for (int dir = 0; dir < 4; dir++) {
            int dx = DIR_DELTA[dir][0];
            int dy = DIR_DELTA[dir][1];
            
            int x1 = x0 + dx;
            int y1 = y0 + dy;
            
            if (is_free_cell(state, x1, y1)) {
                for (int n = 0; n < bottleneck_limit; n++) {
                    int x2 = x1 + dx;
                    int y2 = y1 + dy;
                    
                    if (!is_free_cell(state, x2, y2)) {
                        if (check_chokepoint(state, state->last_color, dir, n + 1)) {
                            return true;
                        }
                        break;
                    }
                    x1 = x2;
                    y1 = y2;
                }
            }
        }
        
        return false;
    }

    // ===== NODES =====

    node_storage_t node_storage_create (size_t max_nodes) {
        node_storage_t storage;
        // FIXED: Proper memory allocation instead of casting integer to pointer
        storage.start = (tree_node_t*)malloc(max_nodes * sizeof(tree_node_t));

        if (!storage.start) {
            printf("cannot initialize memory.\n");
            exit(1);
        }
        storage.capacity = max_nodes;
        storage.count = 0;

        return storage;
    }

    tree_node_t* node_storage_alloc (node_storage_t* storage) {
        if (storage->count >= storage->capacity) {
            return nullptr;
        }

        tree_node_t* rval = storage->start + storage->count;
        ++storage->count;
        return rval;
    }

    void node_storage_unalloc (node_storage_t* storage,
                          const tree_node_t* n) {
        assert( storage->count && n == storage->start + storage->count - 1 );
        --storage->count;
    }

    void node_storage_destroy (node_storage_t* storage) {
        free(storage->start);
    }
    
    tree_node_t* create_node (game_state_t* state, node_storage_t* storage, tree_node_t* parent) {
        tree_node_t* rval = node_storage_alloc(storage);
        if (!rval) { return nullptr; }

        rval->parent = parent;
        rval->cost_to_come = 0;
        rval->cost_to_go = 0;

        memcpy(&(rval->game_state), state, sizeof(game_state_t));
        
        return rval;
    }

    void node_update_costs(tree_node_t* n, size_t action_cost) {
        if (n->parent) {
            n->cost_to_come = n->parent->cost_to_come + action_cost;
        } else {
            n->cost_to_come = 0;
        }

        n->cost_to_go = n->game_state.num_free;
    }

    // ===== VALIDATION =====

    tree_node_t* validate_state (tree_node_t* node, node_storage_t* storage) {
        
        assert(node == storage->start + storage->count - 1);
        game_state_t* node_state = &node->game_state;
        int color, dir;

        // if (find_forced_move(node_state, &color, &dir)) {
        //     if (!game_can_move(node_state, color, dir)) {
        //         assert(node == storage->start + storage->count - 1);
        //         node_storage_unalloc(storage, node);
        //         return nullptr;
        //     }
        //     printf("\nforced\n");

        //     tree_node_t* forced_child = create_node(node_state, storage, node);

        //     if (forced_child) {

        //         game_make_move(&forced_child->game_state, color, dir, 1);

        //         node_update_costs(forced_child, 0);
        //         forced_child = validate_state(forced_child, storage);
            
        //         if (!forced_child) {
        //             assert(node == storage->start + storage->count - 1);
        //             node_storage_unalloc(storage, node);
        //             return nullptr;
        //         } else {
        //             return forced_child;
        //         }
        //     }
        // }

        if (game_check_deadends(node_state)) {
            assert(node == storage->start + storage->count - 1);
            node_storage_unalloc(storage, node);
            printf("\ndeadend\n");
            return nullptr;
        }

        uint8_t rmap[MAX_CELLS];
        size_t rcount = build_regions(node_state, rmap);
        
        // if (game_regions_stranded(node_state, rcount, rmap, MAX_COLORS, 1)) {
        //     print_board(node_state);
        //     assert(node == storage->start + storage->count - 1);
        //     node_storage_unalloc(storage, node);
        //     printf("\nstranded\n");
        //     return nullptr;
        // }

        // if (check_bottleneck(node_state)) {
        //     assert(node == storage->start + storage->count - 1);
        //     node_storage_unalloc(storage, node);
        //     printf("\nbottleneck\n");
        //     return nullptr;
        // }

        return node;
    }

    // ===== SEARCH =====

    game_state_t* game_search (size_t max_nodes = 100000) {
        game_state_t init_state;
        // print_board(&init_state);
        memset(&init_state, 0, sizeof(game_state_t));
        
        // Initialize state with endpoints
        for (size_t color = 0; color < num_colors; color++) {
            init_state.pos[color] = init_pos[color];
            init_state.cells[init_pos[color]] = create_cell(INIT, color, 0);
            init_state.cells[goal_pos[color]] = create_cell(GOAL, color, 0);
        }
        
        init_state.num_free = size * size - 2 * num_colors;
        init_state.last_color = -1;
        init_state.colors_unsolved = num_colors;

        // print_board(&init_state);
        
        node_storage_t storage = node_storage_create(max_nodes);
    
        
        tree_node_t* root = create_node(&init_state, &storage, nullptr);
        if (!root) {
            node_storage_destroy(&storage);
            return nullptr;
        }
        
        node_update_costs(root, 0);
        
        std::priority_queue<tree_node_t*, std::vector<tree_node_t*>, node_compare> pq;
        
        root = validate_state(root, &storage);
        if (!root) {
            node_storage_destroy(&storage);
            return nullptr;
        }
        int color = next_color_to_move(&root->game_state);
        printf("color to move: %d\n", color);
        
        // print_board(&root->game_state);
        pq.push(root);

        while (!pq.empty()) {
            tree_node_t* node = pq.top();
            pq.pop();

        
            
            game_state_t* state = &node->game_state;
            printf("BOARD JUST POPPED\n");
            print_board(state);
            
            int color = next_color_to_move(state);
            printf("next_color_to_move: last_color=%d, completed=0x%x numfree=%d\n", 
                state->last_color, state->completed, state->num_free);
            if (color < 0) {printf("color less than 0\n"); printf("%d\n", state->colors_unsolved);continue;}
            
            for (int dir = 0; dir < 4; dir++) {
                if (!game_can_move(state, color, dir)) { printf("cannot make move %d\n", dir); print_board(state); continue; }
                
                tree_node_t* child = create_node(state, &storage, node);
                if (!child) {
                    node_storage_destroy(&storage);
                    return nullptr;
                }

                // bool forced = find_forced_move(&child->game_state, &color, &dir);

                size_t action_cost = game_make_move(&child->game_state, color, dir, false);

                node_update_costs(child, action_cost);
                child = validate_state(child, &storage);
                
                if (child) {
                    print_board(&child->game_state);
                    const game_state_t* child_state = &child->game_state;
                    
                    if ( child_state->num_free == 0 && 
                        child_state->completed == (1 << num_colors) - 1 ) {
                        solution = (game_state_t*)malloc(sizeof(game_state_t));
                        memcpy(solution, child_state, sizeof(game_state_t));
                        break;  
                    }
                    pq.push(child);
                    printf("added to queue\n");
                }

            }

            if (solution) break;
        }

        node_storage_destroy(&storage);
        return solution;
    }



    // ===== INITIALIZATION HELPERS =====

    void set_color_endpoints(game_state_t* state, int color, pos_t init, pos_t goal) {
        assert(color < (int)num_colors);
        init_pos[color] = init;
        goal_pos[color] = goal;
        state->pos[color] = init;
        state->cells[init] = create_cell(INIT, color, 0);
        state->cells[goal] = create_cell(GOAL, color, 0);
    }

    void set_num_free(game_state_t* state, uint8_t free) {
        state->num_free = free;
    }

    // ===== UTILITY =====

    bool is_solved(game_state_t* state) {
        return state->num_free == 0 && state->completed == ((1 << num_colors) - 1);
    }

    bool read_from_file(const std::string& filename) {
        printf("DEBUG: Starting read_from_file for: %s\n", filename.c_str());
        printf("size: %zu\n", size);
        
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return false;
        }
        printf("DEBUG: File opened successfully\n");

        std::vector<std::string> lines;
        std::string line;
        
        while (std::getline(file, line)) {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            if (!line.empty()) {
                lines.push_back(line);
            }
        }
        file.close();
        printf("DEBUG: Read %zu lines\n", lines.size());

        if (lines.empty()) {
            std::cerr << "Empty file!" << std::endl;
            return false;
        }

        size = lines.size();
        printf("DEBUG: Set size = %zu\n", size);
        
        for (size_t i = 0; i < lines.size(); i++) {
            printf("DEBUG: Line %zu length = %zu\n", i, lines[i].length());
            if (lines[i].length() != size) {
                std::cerr << "Inconsistent line lengths!" << std::endl;
                return false;
            }
        }

        if (size > MAX_SIZE) {
            std::cerr << "Board too large! Max size is " << MAX_SIZE << std::endl;
            return false;
        }

        printf("DEBUG: Resetting state\n");
        num_colors = 0;
        uint8_t color_tbl[128];
        memset(color_tbl, 0xff, sizeof(color_tbl));
        memset(init_pos, 0xff, sizeof(init_pos));
        memset(goal_pos, 0xff, sizeof(goal_pos));
        printf("DEBUG: State reset complete\n");

        printf("DEBUG: Starting board parse\n");
        for (size_t y = 0; y < size; y++) {
            printf("DEBUG: Processing row %zu\n", y);
            for (size_t x = 0; x < size; x++) {
                char c = lines[y][x];
                
                if (std::isalpha(c)) {
                    printf("DEBUG: Found letter '%c' at (%zu, %zu)\n", c, x, y);
                    
                    pos_t pos = pos_from_coords(x, y);
                    printf("DEBUG: pos_from_coords returned: %u\n", pos);
                    
                    uint8_t color = color_tbl[static_cast<uint8_t>(c)];
                    printf("DEBUG: color_tbl lookup returned: %u\n", color);
                    
                    if (color == 0xff) {
                        printf("DEBUG: New color, num_colors = %zu\n", num_colors);
                        
                        if (num_colors >= MAX_COLORS) {
                            std::cerr << "Too many colors! Max is " << MAX_COLORS << std::endl;
                            return false;
                        }
                        
                        color = num_colors;
                        color_tbl[static_cast<uint8_t>(c)] = color;
                        init_pos[color] = pos;
                        printf("DEBUG: Set init_pos[%u] = %u\n", color, pos);
                        num_colors++;
                        
                    } else {
                        printf("DEBUG: Second occurrence of color %u\n", color);
                        
                        if (goal_pos[color] != INVALID_POS) {
                            std::cerr << "Color " << c << " appears more than twice!" << std::endl;
                            return false;
                        }
                        goal_pos[color] = pos;
                        printf("DEBUG: Set goal_pos[%u] = %u\n", color, pos);
                    }
                }
            }
        }

        printf("DEBUG: Verifying endpoints, num_colors = %zu\n", num_colors);
        for (size_t i = 0; i < num_colors; i++) {
            printf("DEBUG: Color %zu: init=%u, goal=%u\n", i, init_pos[i], goal_pos[i]);
            if (goal_pos[i] == INVALID_POS) {
                std::cerr << "Color missing endpoint!" << std::endl;
                return false;
            }
        }

        printf("DEBUG: About to call order_colors()\n");
        fflush(stdout);  // Force flush before potential crash
        
        order_colors();
        
        printf("DEBUG: order_colors() completed\n");

        std::cout << "Successfully loaded " << size << "x" << size 
                << " board with " << num_colors << " colors" << std::endl;
        
        return true;
    }

    void print_board(game_state_t* state) {
        for (size_t y = 0; y < size; y++) {
            for (size_t x = 0; x < size; x++) {
                pos_t p = pos_from_coords(x, y);
                cell_t c = state->cells[p];
                
                if (c == 0) {
                    printf(".");
                } else {
                    int type = cell_get_type(c);
                    int color = cell_get_color(c);
                    
                    if (type == INIT) printf("%c", 'A' + color);
                    else if (type == GOAL) printf("%c", 'a' + color);
                    else printf("%d", color);
                }
                printf(" ");
            }
            printf("\n");
        }
    }

};

int main() {
    FlowSolver solver;
    
    if (solver.read_from_file("puzzles/easy_3x3.txt")) {
        solver.game_search();
        if (solver.solution) {
            std::cout << "Solution found!" << std::endl;
            solver.print_board(solver.solution);
            free(solver.solution);  // Don't forget to free!
        } else {
            std::cout << "No solution found." << std::endl;
        }
    }
    return 0;
}