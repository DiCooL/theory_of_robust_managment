def saturation(u: float, limit: float | None) -> float:
    if limit is None:
        return u
    return min(max(u, -limit), limit)


def dead_zone(u: float, u_minus: float, u_plus: float) -> float:
    if u > u_plus:
        return u - u_plus
    if u < u_minus:
        return u - u_minus
    return 0.0


def dead_zone_inverse(v: float, u_minus: float, u_plus: float, band: float = 0.0) -> float:
    if band > 0.0 and abs(v) < band:
        share = abs(v) / band
        return v + (u_plus if v > 0 else u_minus) * share
    if v > 0:
        return v + u_plus
    if v < 0:
        return v + u_minus
    return 0.0


def actuator(u: float, u_minus: float, u_plus: float, limit: float | None) -> float:
    return saturation(dead_zone(u, u_minus, u_plus), limit)


def saturation_excess(u: float, u_minus: float, u_plus: float, limit: float | None) -> float:
    v = dead_zone(u, u_minus, u_plus)
    return saturation(v, limit) - v
