from __future__ import annotations

from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl_x(part, name, radius, length, xyz, material, segments_hint: int | None = None):
    # URDF cylinders are authored along local +Z; rotate them into the slide axis.
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_z(part, name, radius, length, xyz, material):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz), material=material, name=name)


def _circle_profile(radius: float, *, cx: float = 0.0, cy: float = 0.0, segments: int = 24):
    return [(cx + cos(tau * i / segments) * radius, cy + sin(tau * i / segments) * radius) for i in range(segments)]


def _shift_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _access_cover_mesh(name: str, width: float, depth: float, thickness: float, *, slot_width: float):
    outer = rounded_rect_profile(width, depth, radius=min(width, depth) * 0.10, corner_segments=6)
    slot = rounded_rect_profile(width * 0.48, slot_width, radius=slot_width * 0.48, corner_segments=8)
    holes = [slot]
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            holes.append(_circle_profile(min(width, depth) * 0.045, cx=sx * width * 0.38, cy=sy * depth * 0.30))
    return mesh_from_geometry(ExtrudeWithHolesGeometry(outer, holes, thickness, center=True), name)


def _guide_shoe(
    part,
    prefix: str,
    *,
    x: float,
    rail_y: float,
    rail_z: float,
    rail_radius: float,
    length: float,
    cap_width: float,
    cap_height: float,
    cap_bottom: float,
    cheek_width: float,
    cheek_height: float,
    tunnel_half_width: float,
    shoe_material,
    wear_material,
):
    """A rigid bridge-style linear bearing block with visible side cheeks around a rail."""
    cap_center_z = cap_bottom + cap_height * 0.5
    cheek_center_z = cap_bottom + 0.001 - cheek_height * 0.5
    cheek_offset = tunnel_half_width + cheek_width * 0.5
    liner_width = max(0.004, tunnel_half_width - rail_radius)
    liner_offset = rail_radius + liner_width * 0.5

    _box(part, f"{prefix}_cap", (length, cap_width, cap_height), (x, rail_y, cap_center_z), shoe_material)
    _box(
        part,
        f"{prefix}_cheek_0",
        (length, cheek_width, cheek_height),
        (x, rail_y - cheek_offset, cheek_center_z),
        shoe_material,
    )
    _box(
        part,
        f"{prefix}_cheek_1",
        (length, cheek_width, cheek_height),
        (x, rail_y + cheek_offset, cheek_center_z),
        shoe_material,
    )
    # Thin bronze liners are proud of the cheeks but stay clear of the guide rail.
    _box(
        part,
        f"{prefix}_liner_0",
        (length * 0.72, liner_width, cheek_height * 0.62),
        (x, rail_y - liner_offset, cheek_center_z),
        wear_material,
    )
    _box(
        part,
        f"{prefix}_liner_1",
        (length * 0.72, liner_width, cheek_height * 0.62),
        (x, rail_y + liner_offset, cheek_center_z),
        wear_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_prismatic_chain")

    dark_plate = model.material("dark_hardcoat", rgba=(0.10, 0.115, 0.12, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.03, 0.035, 0.035, 1.0))
    rail_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.33, 0.35, 0.36, 1.0))
    bronze = model.material("bronze_wear", rgba=(0.78, 0.55, 0.25, 1.0))
    stop_red = model.material("red_stop", rgba=(0.64, 0.06, 0.04, 1.0))

    base_cover_mesh = _access_cover_mesh("base_access_cover", 0.220, 0.074, 0.006, slot_width=0.018)
    lower_cover_mesh = _access_cover_mesh("lower_access_cover", 0.205, 0.064, 0.006, slot_width=0.016)
    upper_cover_mesh = _access_cover_mesh("upper_access_cover", 0.175, 0.064, 0.006, slot_width=0.014)

    base = model.part("base")
    _box(base, "base_plate", (0.960, 0.360, 0.035), (0.0, 0.0, 0.0175), dark_plate)
    _box(base, "base_front_tie", (0.065, 0.340, 0.055), (-0.440, 0.0, 0.043), black_oxide)
    _box(base, "base_rear_tie", (0.065, 0.340, 0.055), (0.440, 0.0, 0.043), black_oxide)
    _box(base, "datum_bar_0", (0.830, 0.018, 0.018), (0.0, -0.165, 0.047), satin_steel)
    _box(base, "datum_bar_1", (0.830, 0.018, 0.018), (0.0, 0.165, 0.047), satin_steel)
    base.visual(base_cover_mesh, origin=Origin(xyz=(-0.070, 0.0, 0.038)), material=cover_gray, name="base_cover")
    for ix, sx in enumerate((-1.0, 1.0)):
        for iy, sy in enumerate((-1.0, 1.0)):
            _cyl_z(base, f"base_cover_screw_{ix}_{iy}", 0.007, 0.004, (-0.070 + sx * 0.084, sy * 0.022, 0.043), satin_steel)

    for rail_index, rail_y in enumerate((-0.105, 0.105)):
        _cyl_x(base, f"base_rail_{rail_index}", 0.014, 0.860, (0.0, rail_y, 0.070), rail_steel)
        for support_index, support_x in enumerate((-0.315, 0.0, 0.315)):
            _box(
                base,
                f"base_rail_saddle_{rail_index}_{support_index}",
                (0.082, 0.040, 0.025),
                (support_x, rail_y, 0.0475),
                black_oxide,
            )
            _box(
                base,
                f"base_saddle_cap_{rail_index}_{support_index}",
                (0.050, 0.052, 0.010),
                (support_x, rail_y, 0.0595),
                satin_steel,
            )
        for end_index, end_x in enumerate((-0.438, 0.438)):
            _cyl_x(base, f"base_stop_collar_{rail_index}_{end_index}", 0.020, 0.020, (end_x, rail_y, 0.070), stop_red)
            _box(
                base,
                f"base_stop_key_{rail_index}_{end_index}",
                (0.026, 0.052, 0.024),
                (end_x, rail_y, 0.050),
                stop_red,
            )

    lower_stage = model.part("lower_stage")
    _box(lower_stage, "lower_carriage_plate", (0.560, 0.258, 0.024), (0.0, 0.0, 0.128), dark_plate)
    _box(lower_stage, "lower_front_crossbar", (0.024, 0.260, 0.052), (-0.270, 0.0, 0.153), black_oxide)
    _box(lower_stage, "lower_rear_crossbar", (0.024, 0.260, 0.052), (0.270, 0.0, 0.153), black_oxide)
    lower_stage.visual(lower_cover_mesh, origin=Origin(xyz=(-0.035, 0.0, 0.143)), material=cover_gray, name="lower_cover")
    for ix, sx in enumerate((-1.0, 1.0)):
        for iy, sy in enumerate((-1.0, 1.0)):
            _cyl_z(lower_stage, f"lower_cover_screw_{ix}_{iy}", 0.006, 0.004, (-0.035 + sx * 0.078, sy * 0.019, 0.148), satin_steel)

    shoe_id = 0
    for rail_y in (-0.105, 0.105):
        for shoe_x in (-0.170, 0.170):
            _guide_shoe(
                lower_stage,
                f"lower_shoe_{shoe_id}",
                x=shoe_x,
                rail_y=rail_y,
                rail_z=0.070,
                rail_radius=0.014,
                length=0.110,
                cap_width=0.080,
                cap_height=0.030,
                cap_bottom=0.086,
                cheek_width=0.015,
                cheek_height=0.034,
                tunnel_half_width=0.024,
                shoe_material=black_oxide,
                wear_material=bronze,
            )
            shoe_id += 1

    for rail_index, rail_y in enumerate((-0.070, 0.070)):
        _box(lower_stage, f"upper_rail_bed_{rail_index}", (0.560, 0.024, 0.029), (0.0, rail_y, 0.1545), black_oxide)
        _cyl_x(lower_stage, f"upper_rail_{rail_index}", 0.010, 0.560, (0.0, rail_y, 0.178), rail_steel)
        for pad_index, pad_x in enumerate((-0.230, 0.230)):
            _box(
                lower_stage,
                f"upper_rail_clamp_{rail_index}_{pad_index}",
                (0.046, 0.040, 0.017),
                (pad_x, rail_y, 0.163),
                satin_steel,
            )
        for end_index, end_x in enumerate((-0.282, 0.282)):
            _cyl_x(lower_stage, f"upper_stop_collar_{rail_index}_{end_index}", 0.0155, 0.016, (end_x, rail_y, 0.178), stop_red)
    _box(lower_stage, "lower_alignment_plate_0", (0.145, 0.018, 0.014), (0.0, -0.137, 0.147), satin_steel)
    _box(lower_stage, "lower_alignment_plate_1", (0.145, 0.018, 0.014), (0.0, 0.137, 0.147), satin_steel)
    _box(lower_stage, "lower_stop_tab", (0.030, 0.150, 0.028), (-0.242, 0.0, 0.102), stop_red)

    upper_stage = model.part("upper_stage")
    _box(upper_stage, "upper_saddle_plate", (0.360, 0.184, 0.022), (0.0, 0.0, 0.231), dark_plate)
    _box(upper_stage, "upper_alignment_plate", (0.285, 0.122, 0.012), (0.0, 0.0, 0.248), satin_steel)
    upper_stage.visual(upper_cover_mesh, origin=Origin(xyz=(0.040, 0.0, 0.257)), material=cover_gray, name="upper_cover")
    for ix, sx in enumerate((-1.0, 1.0)):
        for iy, sy in enumerate((-1.0, 1.0)):
            _cyl_z(upper_stage, f"upper_cover_screw_{ix}_{iy}", 0.0055, 0.004, (0.040 + sx * 0.066, sy * 0.019, 0.262), satin_steel)
    _box(upper_stage, "upper_front_bracket", (0.032, 0.172, 0.050), (-0.175, 0.0, 0.242), black_oxide)
    _box(upper_stage, "upper_rear_bracket", (0.032, 0.172, 0.050), (0.175, 0.0, 0.242), black_oxide)
    _box(upper_stage, "upper_end_stop", (0.028, 0.126, 0.022), (-0.148, 0.0, 0.198), stop_red)

    shoe_id = 0
    for rail_y in (-0.070, 0.070):
        for shoe_x in (-0.120, 0.120):
            _guide_shoe(
                upper_stage,
                f"upper_shoe_{shoe_id}",
                x=shoe_x,
                rail_y=rail_y,
                rail_z=0.178,
                rail_radius=0.010,
                length=0.085,
                cap_width=0.066,
                cap_height=0.026,
                cap_bottom=0.194,
                cheek_width=0.013,
                cheek_height=0.032,
                tunnel_half_width=0.020,
                shoe_material=black_oxide,
                wear_material=bronze,
            )
            shoe_id += 1

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_stage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.32, lower=0.0, upper=0.155),
        motion_properties=MotionProperties(damping=18.0, friction=6.0),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=upper_stage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.25, lower=0.0, upper=0.095),
        motion_properties=MotionProperties(damping=14.0, friction=4.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    base_slide = object_model.get_articulation("base_slide")
    stage_slide = object_model.get_articulation("stage_slide")

    prismatic_joints = [joint for joint in object_model.articulations if joint.articulation_type == ArticulationType.PRISMATIC]
    ctx.check(
        "two serial prismatic joints",
        len(object_model.articulations) == 2 and len(prismatic_joints) == 2,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "both slide axes are longitudinal",
        tuple(base_slide.axis) == (1.0, 0.0, 0.0) and tuple(stage_slide.axis) == (1.0, 0.0, 0.0),
        details=f"base_axis={base_slide.axis}, stage_axis={stage_slide.axis}",
    )

    ctx.expect_overlap(
        lower_stage,
        base,
        axes="x",
        elem_a="lower_shoe_1_cap",
        elem_b="base_rail_0",
        min_overlap=0.105,
        name="lower stage shoe remains long on base rail at rest",
    )
    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        positive_elem="lower_shoe_1_cap",
        negative_elem="base_rail_0",
        min_gap=0.001,
        max_gap=0.004,
        name="lower shoe cap clears base rail while bearing liners carry it",
    )
    ctx.expect_overlap(
        upper_stage,
        lower_stage,
        axes="x",
        elem_a="upper_shoe_1_cap",
        elem_b="upper_rail_0",
        min_overlap=0.080,
        name="upper stage shoe remains long on lower-stage rail at rest",
    )
    ctx.expect_gap(
        upper_stage,
        lower_stage,
        axis="z",
        positive_elem="upper_shoe_1_cap",
        negative_elem="upper_rail_0",
        min_gap=0.004,
        max_gap=0.008,
        name="upper shoe cap clears lower-stage rail while liners carry it",
    )

    lower_rest = ctx.part_world_position(lower_stage)
    upper_rest = ctx.part_world_position(upper_stage)
    with ctx.pose({base_slide: base_slide.motion_limits.upper, stage_slide: stage_slide.motion_limits.upper}):
        ctx.expect_overlap(
            lower_stage,
            base,
            axes="x",
            elem_a="lower_shoe_1_cap",
            elem_b="base_rail_0",
            min_overlap=0.080,
            name="lower stage keeps retained rail engagement at full travel",
        )
        ctx.expect_overlap(
            upper_stage,
            lower_stage,
            axes="x",
            elem_a="upper_shoe_1_cap",
            elem_b="upper_rail_0",
            min_overlap=0.065,
            name="upper stage keeps retained rail engagement at full travel",
        )
        lower_extended = ctx.part_world_position(lower_stage)
        upper_extended = ctx.part_world_position(upper_stage)

    ctx.check(
        "lower stage extends in positive X",
        lower_rest is not None and lower_extended is not None and lower_extended[0] > lower_rest[0] + 0.14,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )
    ctx.check(
        "upper stage adds serial positive X travel",
        upper_rest is not None and upper_extended is not None and upper_extended[0] > upper_rest[0] + 0.23,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    return ctx.report()


object_model = build_object_model()
