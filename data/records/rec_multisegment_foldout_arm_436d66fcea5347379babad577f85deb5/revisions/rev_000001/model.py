from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def y_cylinder(radius: float, length: float, x: float, z: float, y: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length, both=True)
        .translate((0.0, y, 0.0))
    )


def centered_box(
    sx: float,
    sy: float,
    sz: float,
    x: float,
    y: float,
    z: float,
    angle_y_deg: float = 0.0,
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(sx, sy, sz)
    if abs(angle_y_deg) > 1e-9:
        solid = solid.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_y_deg)
    return solid.translate((x, y, z))


def bridge_block(
    span_y: float,
    x: float,
    z: float,
    size_x: float,
    size_z: float,
) -> cq.Workplane:
    return centered_box(size_x, span_y, size_z, x, 0.0, z)


def try_fillet(shape: cq.Workplane, selector: str, radius: float) -> cq.Workplane:
    try:
        return shape.edges(selector).fillet(radius)
    except Exception:
        return shape


def side_plate(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    y_center: float,
    outward_sign: float,
    plate_t: float,
    lug_r: float,
    waist_h: float,
    lightening_h: float,
    emboss_h: float,
    emboss_t: float,
    hole_r: float,
    add_heads_at: tuple[tuple[float, float], ...] = (),
    head_r: float = 0.0,
    head_t: float = 0.0,
    start_lug: bool = True,
    end_lug: bool = True,
    start_hole: bool = True,
    end_hole: bool = True,
) -> cq.Workplane:
    x0, z0 = start
    x1, z1 = end
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    angle = math.degrees(math.atan2(dz, dx))
    mid_x = 0.5 * (x0 + x1)
    mid_z = 0.5 * (z0 + z1)

    plate = centered_box(length, plate_t, waist_h, mid_x, y_center, mid_z, angle)
    if start_lug:
        plate = plate.union(y_cylinder(lug_r, plate_t, x0, z0, y_center))
    if end_lug:
        plate = plate.union(y_cylinder(lug_r, plate_t, x1, z1, y_center))

    if length > 0.09 and lightening_h > 0.0:
        slot = centered_box(
            max(0.04, length - 2.15 * lug_r),
            plate_t + 0.002,
            lightening_h,
            mid_x,
            y_center,
            mid_z,
            angle,
        )
        plate = plate.cut(slot)

    if start_hole:
        plate = plate.cut(y_cylinder(hole_r, plate_t + 0.004, x0, z0, y_center))
    if end_hole:
        plate = plate.cut(y_cylinder(hole_r, plate_t + 0.004, x1, z1, y_center))

    if length > 0.12 and emboss_h > 0.0 and emboss_t > 0.0:
        perp_x = -dz / length
        perp_z = dx / length
        for side in (-1.0, 1.0):
            rib_x = mid_x + perp_x * side * (0.31 * waist_h)
            rib_z = mid_z + perp_z * side * (0.31 * waist_h)
            rib = centered_box(
                max(0.03, 0.56 * length),
                emboss_t,
                emboss_h,
                rib_x,
                y_center + outward_sign * (0.5 * plate_t + 0.5 * emboss_t),
                rib_z,
                angle,
            )
            plate = plate.union(rib)

    if head_r > 0.0 and head_t > 0.0:
        for xh, zh in add_heads_at:
            plate = plate.union(
                y_cylinder(
                    head_r,
                    head_t,
                    xh,
                    zh,
                    y_center + outward_sign * (0.5 * plate_t + 0.5 * head_t),
                )
            )

    return plate


def link_pair(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    outer_width: float,
    plate_t: float,
    lug_r: float,
    waist_h: float,
    lightening_h: float,
    emboss_h: float,
    emboss_t: float,
    hole_r: float,
    add_heads_at: tuple[tuple[float, float], ...] = (),
    head_r: float = 0.0,
    head_t: float = 0.0,
    start_lug: bool = True,
    end_lug: bool = True,
    start_hole: bool = True,
    end_hole: bool = True,
) -> cq.Workplane:
    y_center = 0.5 * (outer_width - plate_t)
    left = side_plate(
        start,
        end,
        y_center=y_center,
        outward_sign=1.0,
        plate_t=plate_t,
        lug_r=lug_r,
        waist_h=waist_h,
        lightening_h=lightening_h,
        emboss_h=emboss_h,
        emboss_t=emboss_t,
        hole_r=hole_r,
        add_heads_at=add_heads_at,
        head_r=head_r,
        head_t=head_t,
        start_lug=start_lug,
        end_lug=end_lug,
        start_hole=start_hole,
        end_hole=end_hole,
    )
    right = side_plate(
        start,
        end,
        y_center=-y_center,
        outward_sign=-1.0,
        plate_t=plate_t,
        lug_r=lug_r,
        waist_h=waist_h,
        lightening_h=lightening_h,
        emboss_h=emboss_h,
        emboss_t=emboss_t,
        hole_r=hole_r,
        add_heads_at=add_heads_at,
        head_r=head_r,
        head_t=head_t,
        start_lug=start_lug,
        end_lug=end_lug,
        start_hole=start_hole,
        end_hole=end_hole,
    )
    return left.union(right)


def center_tongue(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    width_y: float,
    body_h: float,
    barrel_r: float,
    barrel_len: float,
    hole_r: float,
    head_r: float = 0.0,
    head_t: float = 0.0,
) -> cq.Workplane:
    x0, z0 = start
    x1, z1 = end
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    angle = math.degrees(math.atan2(dz, dx))
    mid_x = 0.5 * (x0 + x1)
    mid_z = 0.5 * (z0 + z1)

    tongue = centered_box(length, width_y, body_h, mid_x, 0.0, mid_z, angle)
    tongue = tongue.union(y_cylinder(barrel_r, barrel_len, x0, z0))
    tongue = tongue.cut(y_cylinder(hole_r, barrel_len + 0.004, x0, z0))

    if head_r > 0.0 and head_t > 0.0:
        offset = 0.5 * (barrel_len + head_t)
        tongue = tongue.union(y_cylinder(head_r, head_t, x0, z0, offset))
        tongue = tongue.union(y_cylinder(head_r, head_t, x0, z0, -offset))

    return tongue


def make_base() -> cq.Workplane:
    cheek_t = 0.012
    outer_width = 0.124
    y_center = 0.5 * (outer_width - cheek_t)
    inner_gap = outer_width - 2.0 * cheek_t

    base = centered_box(0.22, 0.14, 0.016, -0.065, 0.0, -0.112)
    base = base.union(centered_box(0.085, 0.12, 0.028, -0.12, 0.0, -0.098))
    base = base.union(centered_box(0.055, 0.11, 0.042, -0.03, 0.0, -0.082))

    for sign in (-1.0, 1.0):
        cheek = side_plate(
            (0.0, 0.0),
            (-0.038, -0.086),
            y_center=sign * y_center,
            outward_sign=sign,
            plate_t=cheek_t,
            lug_r=0.028,
            waist_h=0.047,
            lightening_h=0.0,
            emboss_h=0.012,
            emboss_t=0.0016,
            hole_r=0.0102,
            add_heads_at=((0.0, 0.0),),
            head_r=0.0165,
            head_t=0.003,
        )
        base = base.union(cheek)

    base = base.union(bridge_block(inner_gap + cheek_t, -0.016, -0.03, 0.03, 0.032))
    base = base.union(bridge_block(inner_gap + cheek_t, -0.034, -0.058, 0.024, 0.026))
    base = base.union(bridge_block(inner_gap + cheek_t, -0.006, -0.072, 0.034, 0.016))

    pawl = (
        centered_box(0.022, 0.016, 0.015, 0.018, y_center + 0.018, -0.028)
        .union(centered_box(0.016, 0.016, 0.010, 0.029, y_center + 0.018, -0.016, -18.0))
    )
    detent = centered_box(0.018, 0.010, 0.010, -0.002, -(y_center + 0.016), -0.024, 22.0)
    base = base.union(pawl).union(detent)

    return try_fillet(base, "|Y", 0.0016)


def make_link1() -> cq.Workplane:
    outer_width = 0.084
    plate_t = 0.008
    base_inner_gap = 0.100
    y_center = 0.5 * (outer_width - plate_t)

    link = link_pair(
        (0.0, 0.0),
        (0.245, 0.048),
        outer_width=outer_width,
        plate_t=plate_t,
        lug_r=0.022,
        waist_h=0.050,
        lightening_h=0.024,
        emboss_h=0.010,
        emboss_t=0.0014,
        hole_r=0.0092,
        add_heads_at=((0.245, 0.048),),
        head_r=0.014,
        head_t=0.0026,
    )

    link = link.union(y_cylinder(0.0125, base_inner_gap, 0.0, 0.0))
    link = link.union(bridge_block(outer_width, 0.085, 0.017, 0.028, 0.022))
    link = link.union(bridge_block(outer_width, 0.168, 0.032, 0.024, 0.018))
    link = link.union(centered_box(0.020, 0.020, 0.014, 0.038, y_center + 0.010, -0.004, -18.0))

    return try_fillet(link, "|Y", 0.0014)


def make_link2() -> cq.Workplane:
    outer_width = 0.052
    plate_t = 0.006
    link1_inner_gap = 0.068
    link3_inner_gap = 0.060

    link = link_pair(
        (0.0, 0.0),
        (0.148, -0.021),
        outer_width=outer_width,
        plate_t=plate_t,
        lug_r=0.018,
        waist_h=0.032,
        lightening_h=0.014,
        emboss_h=0.007,
        emboss_t=0.0011,
        hole_r=0.0082,
    )

    link = link.union(y_cylinder(0.011, link1_inner_gap, 0.0, 0.0))
    link = link.union(y_cylinder(0.0105, link3_inner_gap, 0.148, -0.021))
    link = link.union(bridge_block(outer_width, 0.074, -0.010, 0.020, 0.015))

    return try_fillet(link, "|Y", 0.0012)


def make_link3() -> cq.Workplane:
    outer_width = 0.074
    plate_t = 0.007
    y_center = 0.5 * (outer_width - plate_t)

    link = link_pair(
        (0.0, 0.0),
        (0.225, 0.041),
        outer_width=outer_width,
        plate_t=plate_t,
        lug_r=0.019,
        waist_h=0.038,
        lightening_h=0.018,
        emboss_h=0.008,
        emboss_t=0.0011,
        hole_r=0.0084,
        add_heads_at=((0.0, 0.0), (0.225, 0.041)),
        head_r=0.013,
        head_t=0.0022,
    )

    link = link.union(bridge_block(outer_width, 0.086, 0.013, 0.022, 0.015))
    link = link.union(bridge_block(outer_width, 0.155, 0.026, 0.020, 0.013))
    link = link.union(centered_box(0.018, 0.016, 0.010, 0.192, y_center + 0.010, 0.036, -14.0))

    return try_fillet(link, "|Y", 0.0011)


def make_tray() -> cq.Workplane:
    link3_inner_gap = 0.060

    tray = y_cylinder(0.0102, link3_inner_gap, 0.0, 0.0)
    tray = tray.union(centered_box(0.036, 0.024, 0.020, 0.020, 0.0, 0.010, -16.0))
    tray = tray.union(centered_box(0.085, 0.074, 0.006, 0.078, 0.0, 0.028))
    tray = tray.union(centered_box(0.085, 0.006, 0.020, 0.078, 0.034, 0.038))
    tray = tray.union(centered_box(0.085, 0.006, 0.020, 0.078, -0.034, 0.038))
    tray = tray.union(centered_box(0.006, 0.074, 0.018, 0.118, 0.0, 0.037))
    tray = tray.union(centered_box(0.020, 0.010, 0.018, 0.044, 0.026, 0.016, -18.0))
    tray = tray.union(centered_box(0.020, 0.010, 0.018, 0.044, -0.026, 0.016, -18.0))
    tray = tray.union(centered_box(0.024, 0.030, 0.010, 0.045, 0.0, 0.045))

    return try_fillet(tray, "|Y", 0.001)


def make_base_v2() -> cq.Workplane:
    cheek_t = 0.012
    outer_width = 0.124
    y_center = 0.5 * (outer_width - cheek_t)
    inner_gap = outer_width - 2.0 * cheek_t

    base = centered_box(0.238, 0.150, 0.014, -0.086, 0.0, -0.118)
    base = base.union(centered_box(0.102, 0.128, 0.030, -0.126, 0.0, -0.098))
    base = base.union(centered_box(0.054, 0.112, 0.040, -0.050, 0.0, -0.078))

    for sign in (-1.0, 1.0):
        cheek = side_plate(
            (0.0, 0.0),
            (-0.042, -0.090),
            y_center=sign * y_center,
            outward_sign=sign,
            plate_t=cheek_t,
            lug_r=0.028,
            waist_h=0.046,
            lightening_h=0.0,
            emboss_h=0.010,
            emboss_t=0.0014,
            hole_r=0.0102,
            add_heads_at=((0.0, 0.0),),
            head_r=0.016,
            head_t=0.0028,
        )
        base = base.union(cheek)

    base = base.union(bridge_block(inner_gap + cheek_t, -0.028, -0.036, 0.028, 0.026))
    base = base.union(bridge_block(inner_gap + cheek_t, -0.046, -0.064, 0.022, 0.020))
    base = base.union(bridge_block(inner_gap + cheek_t, -0.060, -0.084, 0.024, 0.014))
    base = base.union(centered_box(0.020, 0.016, 0.014, -0.014, y_center + 0.020, -0.025))
    base = base.union(centered_box(0.014, 0.016, 0.010, -0.004, y_center + 0.020, -0.015, -18.0))
    base = base.union(centered_box(0.018, 0.010, 0.010, -0.024, -(y_center + 0.016), -0.024, 18.0))

    return try_fillet(base, "|Y", 0.0014)


def make_link1_v2() -> cq.Workplane:
    outer_width = 0.084
    plate_t = 0.008
    base_inner_gap = 0.100

    link = center_tongue(
        (0.0, 0.0),
        (0.060, 0.011),
        width_y=0.024,
        body_h=0.018,
        barrel_r=0.0125,
        barrel_len=base_inner_gap,
        hole_r=0.0092,
        head_r=0.0145,
        head_t=0.0024,
    )
    link = link.union(
        link_pair(
            (0.052, 0.010),
            (0.245, 0.048),
            outer_width=outer_width,
            plate_t=plate_t,
            lug_r=0.0215,
            waist_h=0.044,
            lightening_h=0.018,
            emboss_h=0.009,
            emboss_t=0.0012,
            hole_r=0.0092,
            add_heads_at=((0.245, 0.048),),
            head_r=0.0135,
            head_t=0.0024,
            start_lug=False,
            start_hole=False,
        )
    )
    link = link.union(bridge_block(outer_width, 0.106, 0.020, 0.020, 0.018))
    link = link.union(bridge_block(outer_width, 0.170, 0.032, 0.018, 0.014))
    link = link.union(centered_box(0.016, 0.016, 0.010, 0.088, 0.028, 0.004, -10.0))

    return try_fillet(link, "|Y", 0.0012)


def make_link2_v2() -> cq.Workplane:
    outer_width = 0.060
    plate_t = 0.006
    link1_inner_gap = 0.068

    link = center_tongue(
        (0.0, 0.0),
        (0.036, -0.005),
        width_y=0.020,
        body_h=0.014,
        barrel_r=0.0105,
        barrel_len=link1_inner_gap,
        hole_r=0.0082,
        head_r=0.012,
        head_t=0.002,
    )
    link = link.union(
        link_pair(
            (0.030, -0.004),
            (0.148, -0.021),
            outer_width=outer_width,
            plate_t=plate_t,
            lug_r=0.016,
            waist_h=0.026,
            lightening_h=0.010,
            emboss_h=0.005,
            emboss_t=0.0010,
            hole_r=0.0082,
            add_heads_at=((0.148, -0.021),),
            head_r=0.0115,
            head_t=0.002,
            start_lug=False,
            start_hole=False,
        )
    )
    link = link.union(bridge_block(outer_width, 0.086, -0.011, 0.016, 0.012))

    return try_fillet(link, "|Y", 0.001)


def make_link3_v2() -> cq.Workplane:
    outer_width = 0.074
    plate_t = 0.007
    link2_inner_gap = 0.048

    link = center_tongue(
        (0.0, 0.0),
        (0.042, 0.008),
        width_y=0.018,
        body_h=0.015,
        barrel_r=0.0103,
        barrel_len=link2_inner_gap,
        hole_r=0.0084,
        head_r=0.0125,
        head_t=0.002,
    )
    link = link.union(
        link_pair(
            (0.036, 0.007),
            (0.225, 0.041),
            outer_width=outer_width,
            plate_t=plate_t,
            lug_r=0.0185,
            waist_h=0.034,
            lightening_h=0.014,
            emboss_h=0.006,
            emboss_t=0.0010,
            hole_r=0.0084,
            add_heads_at=((0.225, 0.041),),
            head_r=0.0125,
            head_t=0.002,
            start_lug=False,
            start_hole=False,
        )
    )
    link = link.union(bridge_block(outer_width, 0.086, 0.015, 0.018, 0.012))
    link = link.union(bridge_block(outer_width, 0.150, 0.026, 0.016, 0.010))

    return try_fillet(link, "|Y", 0.001)


def make_tray_v2() -> cq.Workplane:
    link3_inner_gap = 0.060

    tray = center_tongue(
        (0.0, 0.0),
        (0.030, 0.006),
        width_y=0.022,
        body_h=0.014,
        barrel_r=0.0100,
        barrel_len=link3_inner_gap,
        hole_r=0.0080,
        head_r=0.0115,
        head_t=0.002,
    )
    tray = tray.union(centered_box(0.030, 0.020, 0.014, 0.024, 0.0, 0.010, -12.0))
    tray = tray.union(centered_box(0.086, 0.074, 0.0055, 0.082, 0.0, 0.029))
    tray = tray.union(centered_box(0.086, 0.006, 0.018, 0.082, 0.034, 0.039))
    tray = tray.union(centered_box(0.086, 0.006, 0.018, 0.082, -0.034, 0.039))
    tray = tray.union(centered_box(0.006, 0.074, 0.017, 0.122, 0.0, 0.037))
    tray = tray.union(centered_box(0.018, 0.010, 0.016, 0.046, 0.026, 0.014, -16.0))
    tray = tray.union(centered_box(0.018, 0.010, 0.016, 0.046, -0.026, 0.014, -16.0))
    tray = tray.union(centered_box(0.024, 0.030, 0.010, 0.046, 0.0, 0.045))

    return try_fillet(tray, "|Y", 0.001)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_bracket_arm")

    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    zinc = model.material("zinc", rgba=(0.63, 0.65, 0.69, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.42, 0.44, 0.47, 1.0))

    def add_box(
        part_obj,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        *,
        material,
        name: str,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part_obj.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def add_y_cylinder(
        part_obj,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        *,
        material,
        name: str,
    ) -> None:
        part_obj.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    base = model.part("grounded_foot")
    add_box(base, (0.238, 0.150, 0.014), (-0.086, 0.0, -0.118), material=dark_steel, name="foot_plate")
    add_box(base, (0.102, 0.128, 0.030), (-0.126, 0.0, -0.098), material=dark_steel, name="foot_pedestal")
    add_box(base, (0.056, 0.112, 0.040), (-0.050, 0.0, -0.078), material=dark_steel, name="root_housing")
    for sign, label in ((1.0, "left"), (-1.0, "right")):
        add_box(base, (0.062, 0.012, 0.068), (-0.016, sign * 0.056, -0.002), material=dark_steel, name=f"{label}_cheek")
        add_box(base, (0.048, 0.012, 0.036), (-0.050, sign * 0.056, -0.042), material=dark_steel, name=f"{label}_rear_web")
        add_box(base, (0.046, 0.004, 0.010), (-0.020, sign * 0.064, 0.025), material=dark_steel, name=f"{label}_top_lip")
        add_box(base, (0.040, 0.004, 0.010), (-0.024, sign * 0.064, -0.031), material=dark_steel, name=f"{label}_bottom_lip")
        add_y_cylinder(base, 0.0155, 0.004, (0.0, sign * 0.064, 0.0), material=zinc, name=f"{label}_root_bolt_head")
    add_box(base, (0.028, 0.112, 0.018), (-0.050, 0.0, -0.054), material=dark_steel, name="rear_crossbar_upper")
    add_box(base, (0.026, 0.110, 0.014), (-0.072, 0.0, -0.084), material=dark_steel, name="rear_crossbar_lower")
    add_box(base, (0.020, 0.016, 0.014), (-0.016, 0.076, -0.024), material=dark_steel, name="latch_body")
    add_box(base, (0.014, 0.016, 0.010), (-0.006, 0.076, -0.014), material=dark_steel, name="latch_nose", rpy=(0.0, math.radians(-18.0), 0.0))
    add_box(base, (0.018, 0.010, 0.010), (-0.024, -0.072, -0.024), material=dark_steel, name="detent_block", rpy=(0.0, math.radians(18.0), 0.0))
    add_box(base, (0.014, 0.016, 0.010), (-0.016, 0.068, -0.024), material=dark_steel, name="latch_mount")
    add_box(base, (0.014, 0.014, 0.010), (-0.024, -0.064, -0.024), material=dark_steel, name="detent_mount")

    link1 = model.part("long_link_root")
    add_box(link1, (0.066, 0.100, 0.018), (0.023, 0.0, 0.0), material=zinc, name="root_tongue")
    add_box(link1, (0.026, 0.034, 0.020), (0.060, 0.0, 0.010), material=zinc, name="root_transition")
    add_box(link1, (0.086, 0.024, 0.018), (0.098, 0.0, 0.014), material=zinc, name="center_spine")
    for sign, label in ((1.0, "left"), (-1.0, "right")):
        add_box(
            link1,
            (0.190, 0.008, 0.044),
            (0.150, sign * 0.038, 0.029),
            material=zinc,
            name=f"{label}_plate",
            rpy=(0.0, math.radians(11.2), 0.0),
        )
        add_box(
            link1,
            (0.146, 0.004, 0.008),
            (0.150, sign * 0.0415, 0.047),
            material=zinc,
            name=f"{label}_formed_lip",
            rpy=(0.0, math.radians(11.2), 0.0),
        )
        add_y_cylinder(link1, 0.0215, 0.008, (0.245, sign * 0.038, 0.048), material=zinc, name=f"{label}_distal_lug")
        add_y_cylinder(link1, 0.0135, 0.004, (0.245, sign * 0.044, 0.048), material=dark_steel, name=f"{label}_distal_bolt_head")
    add_box(link1, (0.024, 0.084, 0.022), (0.102, 0.0, 0.020), material=zinc, name="mid_bridge_a")
    add_box(link1, (0.020, 0.084, 0.018), (0.164, 0.0, 0.032), material=zinc, name="mid_bridge_b")
    add_box(link1, (0.020, 0.070, 0.018), (0.206, 0.0, 0.041), material=zinc, name="distal_bridge")

    link2 = model.part("short_link_mid")
    add_box(link2, (0.040, 0.068, 0.014), (0.015, 0.0, 0.0), material=dark_steel, name="root_tongue")
    add_box(link2, (0.018, 0.022, 0.016), (0.036, 0.0, -0.004), material=dark_steel, name="root_transition")
    for sign, label in ((1.0, "left"), (-1.0, "right")):
        add_box(
            link2,
            (0.118, 0.006, 0.026),
            (0.090, sign * 0.027, -0.012),
            material=dark_steel,
            name=f"{label}_plate",
            rpy=(0.0, math.radians(-8.2), 0.0),
        )
        add_box(
            link2,
            (0.084, 0.0035, 0.006),
            (0.090, sign * 0.0295, 0.000),
            material=dark_steel,
            name=f"{label}_formed_lip",
            rpy=(0.0, math.radians(-8.2), 0.0),
        )
        add_y_cylinder(link2, 0.016, 0.006, (0.148, sign * 0.027, -0.021), material=dark_steel, name=f"{label}_distal_lug")
        add_y_cylinder(link2, 0.0115, 0.0035, (0.148, sign * 0.03175, -0.021), material=zinc, name=f"{label}_distal_bolt_head")
    add_box(link2, (0.016, 0.060, 0.014), (0.072, 0.0, -0.010), material=dark_steel, name="mid_bridge")

    link3 = model.part("long_link_tip")
    add_box(link3, (0.045, 0.048, 0.015), (0.016, 0.0, 0.0), material=zinc, name="root_tongue")
    add_box(link3, (0.018, 0.018, 0.016), (0.036, 0.0, 0.006), material=zinc, name="root_transition")
    add_box(link3, (0.074, 0.020, 0.016), (0.078, 0.0, 0.014), material=zinc, name="center_spine")
    for sign, label in ((1.0, "left"), (-1.0, "right")):
        add_box(
            link3,
            (0.190, 0.007, 0.032),
            (0.131, sign * 0.0335, 0.024),
            material=zinc,
            name=f"{label}_plate",
            rpy=(0.0, math.radians(10.2), 0.0),
        )
        add_box(
            link3,
            (0.138, 0.004, 0.006),
            (0.131, sign * 0.0365, 0.038),
            material=zinc,
            name=f"{label}_formed_lip",
            rpy=(0.0, math.radians(10.2), 0.0),
        )
        add_y_cylinder(link3, 0.0185, 0.007, (0.225, sign * 0.0335, 0.041), material=zinc, name=f"{label}_distal_lug")
        add_y_cylinder(link3, 0.0125, 0.0035, (0.225, sign * 0.03875, 0.041), material=dark_steel, name=f"{label}_distal_bolt_head")
    add_box(link3, (0.018, 0.074, 0.014), (0.085, 0.0, 0.017), material=zinc, name="mid_bridge_a")
    add_box(link3, (0.016, 0.074, 0.012), (0.150, 0.0, 0.027), material=zinc, name="mid_bridge_b")
    add_box(link3, (0.018, 0.066, 0.014), (0.198, 0.0, 0.036), material=zinc, name="distal_bridge")

    tray = model.part("tray_end_bracket")
    add_box(tray, (0.036, 0.060, 0.014), (0.013, 0.0, 0.0), material=tray_gray, name="root_tongue")
    add_box(tray, (0.022, 0.024, 0.016), (0.028, 0.0, 0.010), material=tray_gray, name="neck_block")
    add_box(tray, (0.046, 0.050, 0.024), (0.050, 0.0, 0.020), material=tray_gray, name="tray_support_core")
    add_box(tray, (0.086, 0.074, 0.0055), (0.082, 0.0, 0.029), material=tray_gray, name="tray_floor")
    add_box(tray, (0.086, 0.006, 0.018), (0.082, 0.034, 0.039), material=tray_gray, name="left_wall")
    add_box(tray, (0.086, 0.006, 0.018), (0.082, -0.034, 0.039), material=tray_gray, name="right_wall")
    add_box(tray, (0.006, 0.074, 0.017), (0.122, 0.0, 0.037), material=tray_gray, name="front_wall")
    add_box(tray, (0.020, 0.010, 0.016), (0.046, 0.026, 0.014), material=tray_gray, name="left_ear", rpy=(0.0, math.radians(-16.0), 0.0))
    add_box(tray, (0.020, 0.010, 0.016), (0.046, -0.026, 0.014), material=tray_gray, name="right_ear", rpy=(0.0, math.radians(-16.0), 0.0))
    add_box(tray, (0.024, 0.030, 0.010), (0.046, 0.0, 0.037), material=tray_gray, name="back_stop")

    model.articulation(
        "foot_to_root_link",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=-0.35, upper=1.05),
    )
    model.articulation(
        "root_to_short_link",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(0.245, 0.0, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=-1.10, upper=0.60),
    )
    model.articulation(
        "short_to_tip_link",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(0.148, 0.0, -0.021)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.55, upper=1.25),
    )
    model.articulation(
        "tip_to_tray",
        ArticulationType.REVOLUTE,
        parent=link3,
        child=tray,
        origin=Origin(xyz=(0.225, 0.0, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.65, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("grounded_foot")
    link1 = object_model.get_part("long_link_root")
    link2 = object_model.get_part("short_link_mid")
    link3 = object_model.get_part("long_link_tip")
    tray = object_model.get_part("tray_end_bracket")
    j1 = object_model.get_articulation("foot_to_root_link")
    j2 = object_model.get_articulation("root_to_short_link")
    j3 = object_model.get_articulation("short_to_tip_link")
    j4 = object_model.get_articulation("tip_to_tray")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_revolute_axes_parallel",
        all(j.axis == (0.0, 1.0, 0.0) for j in (j1, j2, j3, j4)),
        details="expected all four joints to share the same +Y revolute axis",
    )

    ctx.expect_contact(base, link1, name="base_contacts_root_link")
    ctx.expect_contact(link1, link2, name="root_link_contacts_short_link")
    ctx.expect_contact(link2, link3, name="short_link_contacts_tip_link")
    ctx.expect_contact(link3, tray, name="tip_link_contacts_tray")

    with ctx.pose(
        {
            j1: 0.45,
            j2: -0.35,
            j3: 0.55,
            j4: 0.10,
        }
    ):
        ctx.expect_gap(tray, base, axis="x", min_gap=0.18, name="deployed_tray_clears_base_in_x")
        ctx.expect_gap(base, tray, axis="z", min_gap=0.05, name="deployed_tray_hangs_clear_below_foot")

    with ctx.pose(
        {
            j1: -0.20,
            j2: -0.52,
            j3: 0.22,
            j4: -0.35,
        }
    ):
        ctx.expect_gap(
            tray,
            base,
            axis="x",
            min_gap=0.06,
            name="tray_stays_clear_of_grounded_foot",
        )
        ctx.expect_gap(link3, base, axis="z", min_gap=0.01, name="tip_link_stays_above_foot_in_nested_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
