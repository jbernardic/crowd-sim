#include "walls.h"
#include "../config.h"
#include <cmath>

void apply_wall_collision(
    const std::vector<Vector3>&         positions,
    std::vector<Vector3>&               vel,
    const std::unordered_set<uint64_t>& tiles,
    float                               dt)
{
    const auto& cfg = get_wall_config();
    if (!cfg.enabled || tiles.empty()) return;

    const float ts = cfg.tile_size;
    const float r  = get_agent_config().radius;

    for (int i = 0; i < (int)positions.size(); ++i) {
        float px = positions[i].x + vel[i].x * dt;
        float py = positions[i].y + vel[i].y * dt;
        const float ox = px, oy = py;

        int minTx = (int)floorf((px - r) / ts);
        int maxTx = (int)floorf((px + r) / ts);
        int minTy = (int)floorf((py - r) / ts);
        int maxTy = (int)floorf((py + r) / ts);

        for (int tx = minTx; tx <= maxTx; ++tx) {
            for (int ty = minTy; ty <= maxTy; ++ty) {
                if (!tiles.count(encode_tile(tx, ty))) continue;

                float left = tx * ts, right = left + ts;
                float top  = ty * ts, bottom = top  + ts;

                float cx   = fmaxf(left, fminf(px, right));
                float cy   = fmaxf(top,  fminf(py, bottom));
                float dx   = px - cx;
                float dy   = py - cy;
                float dist = sqrtf(dx * dx + dy * dy);

                if (dist > 0.001f && dist < r) {
                    float push = r - dist;
                    px += (dx / dist) * push;
                    py += (dy / dist) * push;
                } else if (dist <= 0.001f) {
                    float dLeft = px - left, dRight  = right  - px;
                    float dTop  = py - top,  dBottom = bottom - py;
                    float minD  = fminf(fminf(dLeft, dRight), fminf(dTop, dBottom));

                    if      (minD == dLeft)   px = left   - r;
                    else if (minD == dRight)  px = right  + r;
                    else if (minD == dTop)    py = top    - r;
                    else                      py = bottom + r;
                }
            }
        }

        vel[i].x += (px - ox) / dt;
        vel[i].y += (py - oy) / dt;
    }
}
