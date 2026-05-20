from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, rpy, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _build_lid(part, *, width: float, frame_material, glass_material, handle_x: float) -> None:
    length = 1.16
    rail = 0.035
    frame_h = 0.035
    half_x = length / 2.0
    half_y = width / 2.0

    _box(
        part,
        "front_frame",
        (length, rail, frame_h),
        (0.0, -half_y + rail / 2.0, 0.0),
        frame_material,
    )
    _box(
        part,
        "rear_frame",
        (length, rail, frame_h),
        (0.0, half_y - rail / 2.0, 0.0),
        frame_material,
    )
    _box(
        part,
        "end_frame_0",
        (rail, width, frame_h),
        (-half_x + rail / 2.0, 0.0, 0.0),
        frame_material,
    )
    _box(
        part,
        "end_frame_1",
        (rail, width, frame_h),
        (half_x - rail / 2.0, 0.0, 0.0),
        frame_material,
    )
    # The glass underlaps the frame lips slightly so the lid reads as a single
    # framed panel rather than a loose floating pane.
    _box(
        part,
        "glass",
        (length - 0.045, width - 0.050, 0.012),
        (0.0, 0.0, -0.006),
        glass_material,
    )
    _box(
        part,
        "pull_handle",
        (0.075, width * 0.42, 0.035),
        (handle_x, 0.0, 0.017),
        Material("warm_gray_handle", rgba=(0.05, 0.055, 0.06, 1.0)),
    )


def _build_caster_support(part, metal, dark) -> None:
    _box(part, "swivel_plate", (0.16, 0.12, 0.012), (0.0, 0.0, -0.006), metal)
    _cyl(part, "swivel_stem", 0.018, 0.070, (0.0, 0.0, -0.047), (0.0, 0.0, 0.0), metal)
    _box(part, "fork_bridge", (0.13, 0.11, 0.020), (0.0, 0.0, -0.072), metal)
    _box(part, "fork_cheek_0", (0.090, 0.012, 0.130), (0.0, -0.047, -0.135), metal)
    _box(part, "fork_cheek_1", (0.090, 0.012, 0.130), (0.0, 0.047, -0.135), metal)
    _box(part, "axle_cap_0", (0.030, 0.012, 0.030), (0.0, -0.059, -0.140), dark)
    _box(part, "axle_cap_1", (0.030, 0.012, 0.030), (0.0, 0.059, -0.140), dark)


def _build_wheel(part, rubber, hub) -> None:
    _cyl(part, "tire", 0.055, 0.082, (0.0, 0.0, 0.0), (pi / 2.0, 0.0, 0.0), rubber)
    _cyl(part, "hub", 0.026, 0.055, (0.0, 0.0, 0.0), (pi / 2.0, 0.0, 0.0), hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supermarket_island_display_freezer")

    enamel = Material("white_powder_coated_steel", rgba=(0.92, 0.94, 0.93, 1.0))
    liner = Material("slightly_warm_inner_liner", rgba=(0.82, 0.86, 0.86, 1.0))
    insulation_edge = Material("pale_gray_insulated_edge", rgba=(0.73, 0.77, 0.78, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
    black = Material("black_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    wire = Material("coated_wire", rgba=(0.80, 0.84, 0.84, 1.0))
    glass = Material("slightly_blue_glass", rgba=(0.60, 0.86, 1.0, 0.34))
    blue = Material("retail_blue_accent", rgba=(0.05, 0.22, 0.60, 1.0))
    shadow = Material("dark_recess_shadow", rgba=(0.045, 0.055, 0.060, 1.0))

    tub = model.part("tub")

    # Long insulated cabinet and open tub.  The body is built from walls and a
    # liner floor so the visible top remains a genuine cavity, not a solid slab.
    _box(tub, "lower_plinth", (2.52, 1.04, 0.18), (0.0, 0.0, 0.29), enamel)
    _box(tub, "shadow_kick", (2.28, 0.88, 0.050), (0.0, 0.0, 0.225), shadow)
    _box(tub, "liner_floor", (2.26, 0.78, 0.060), (0.0, 0.0, 0.410), liner)
    _box(tub, "front_wall", (2.52, 0.12, 0.57), (0.0, -0.51, 0.665), enamel)
    _box(tub, "rear_wall", (2.52, 0.12, 0.57), (0.0, 0.51, 0.665), enamel)
    _box(tub, "end_wall_0", (0.12, 1.04, 0.57), (-1.20, 0.0, 0.665), enamel)
    _box(tub, "end_wall_1", (0.12, 1.04, 0.57), (1.20, 0.0, 0.665), enamel)
    _box(tub, "front_inner_liner", (2.28, 0.025, 0.48), (0.0, -0.438, 0.690), liner)
    _box(tub, "rear_inner_liner", (2.28, 0.025, 0.48), (0.0, 0.438, 0.690), liner)
    _box(tub, "end_inner_liner_0", (0.025, 0.78, 0.48), (-1.118, 0.0, 0.690), liner)
    _box(tub, "end_inner_liner_1", (0.025, 0.78, 0.48), (1.118, 0.0, 0.690), liner)

    # Thick rim cap and two stacked longitudinal track levels for the sliding lids.
    _box(tub, "front_rim_cap", (2.58, 0.16, 0.040), (0.0, -0.51, 0.970), insulation_edge)
    _box(tub, "rear_rim_cap", (2.58, 0.16, 0.040), (0.0, 0.51, 0.970), insulation_edge)
    _box(tub, "end_rim_cap_0", (0.16, 1.04, 0.040), (-1.21, 0.0, 0.970), insulation_edge)
    _box(tub, "end_rim_cap_1", (0.16, 1.04, 0.040), (1.21, 0.0, 0.970), insulation_edge)
    _box(tub, "lower_track_front", (2.42, 0.035, 0.018), (0.0, -0.440, 0.988), aluminum)
    _box(tub, "lower_track_rear", (2.42, 0.035, 0.018), (0.0, 0.440, 0.988), aluminum)
    _box(tub, "upper_track_front_riser", (2.42, 0.025, 0.045), (0.0, -0.535, 1.0085), aluminum)
    _box(tub, "upper_track_rear_riser", (2.42, 0.025, 0.045), (0.0, 0.535, 1.0085), aluminum)
    _box(tub, "upper_track_front", (2.42, 0.035, 0.018), (0.0, -0.505, 1.028), aluminum)
    _box(tub, "upper_track_rear", (2.42, 0.035, 0.018), (0.0, 0.505, 1.028), aluminum)
    _box(tub, "front_blue_stripe", (2.28, 0.012, 0.060), (0.0, -0.573, 0.56), blue)
    _box(tub, "rear_blue_stripe", (2.28, 0.012, 0.060), (0.0, 0.573, 0.56), blue)

    # Wire basket floor and dividers, tied into the liner and side walls.
    for i, y in enumerate((-0.36, -0.18, 0.0, 0.18, 0.36)):
        _box(tub, f"basket_floor_long_{i}", (2.22, 0.010, 0.010), (0.0, y, 0.462), wire)
    for i, x in enumerate((-0.90, -0.45, 0.0, 0.45, 0.90)):
        _box(tub, f"basket_floor_cross_{i}", (0.010, 0.88, 0.010), (x, 0.0, 0.462), wire)

    for panel_i, x in enumerate((-0.40, 0.40)):
        for j, z in enumerate((0.54, 0.67, 0.80)):
            _box(tub, f"divider_{panel_i}_horizontal_{j}", (0.012, 0.88, 0.012), (x, 0.0, z), wire)
        for j, y in enumerate((-0.32, -0.16, 0.0, 0.16, 0.32)):
            _box(tub, f"divider_{panel_i}_vertical_{j}", (0.012, 0.012, 0.36), (x, y, 0.67), wire)

    for panel_i, y in enumerate((-0.20, 0.20)):
        for j, z in enumerate((0.55, 0.74)):
            _box(tub, f"length_divider_{panel_i}_rail_{j}", (2.18, 0.012, 0.012), (0.0, y, z), wire)
        for j, x in enumerate((-0.80, -0.40, 0.0, 0.40, 0.80)):
            _box(tub, f"length_divider_{panel_i}_post_{j}", (0.012, 0.012, 0.32), (x, y, 0.64), wire)

    lower_lid = model.part("glass_lid_0")
    _build_lid(lower_lid, width=0.88, frame_material=aluminum, glass_material=glass, handle_x=0.42)
    upper_lid = model.part("glass_lid_1")
    _build_lid(upper_lid, width=0.99, frame_material=aluminum, glass_material=glass, handle_x=-0.42)

    model.articulation(
        "tub_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_lid,
        origin=Origin(xyz=(-0.60, 0.0, 1.0145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.45, lower=0.0, upper=0.45),
    )
    model.articulation(
        "tub_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_lid,
        origin=Origin(xyz=(0.60, 0.0, 1.0545)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.45, lower=0.0, upper=0.45),
    )

    caster_positions = (
        (-1.05, -0.38),
        (-1.05, 0.38),
        (1.05, -0.38),
        (1.05, 0.38),
    )
    for i, (x, y) in enumerate(caster_positions):
        caster = model.part(f"caster_{i}")
        _build_caster_support(caster, aluminum, black)
        wheel = model.part(f"wheel_{i}")
        _build_wheel(wheel, black, aluminum)

        model.articulation(
            f"tub_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=tub,
            child=caster,
            origin=Origin(xyz=(x, y, 0.200)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )
        model.articulation(
            f"caster_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.140)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tub = object_model.get_part("tub")
    lower_lid = object_model.get_part("glass_lid_0")
    upper_lid = object_model.get_part("glass_lid_1")
    lower_slide = object_model.get_articulation("tub_to_lid_0")
    upper_slide = object_model.get_articulation("tub_to_lid_1")

    ctx.expect_gap(
        upper_lid,
        lower_lid,
        axis="z",
        min_gap=0.004,
        max_gap=0.010,
        positive_elem="front_frame",
        negative_elem="front_frame",
        name="stacked lid tracks separate the two glass panels vertically",
    )
    ctx.expect_gap(
        upper_lid,
        lower_lid,
        axis="x",
        min_gap=0.020,
        max_gap=0.060,
        name="closed lids meet with a narrow center seam",
    )
    ctx.expect_overlap(
        lower_lid,
        tub,
        axes="y",
        min_overlap=0.82,
        name="lower glass lid spans the tub opening width",
    )
    ctx.expect_overlap(
        upper_lid,
        tub,
        axes="y",
        min_overlap=0.92,
        name="upper glass lid spans the tub opening width",
    )

    lower_rest = ctx.part_world_position(lower_lid)
    upper_rest = ctx.part_world_position(upper_lid)
    with ctx.pose({lower_slide: 0.45, upper_slide: 0.45}):
        lower_open = ctx.part_world_position(lower_lid)
        upper_open = ctx.part_world_position(upper_lid)
        ctx.expect_gap(
            upper_lid,
            lower_lid,
            axis="z",
            min_gap=0.004,
            max_gap=0.010,
            positive_elem="front_frame",
            negative_elem="front_frame",
            name="overlapping open lids remain on separate stacked rails",
        )
        ctx.expect_overlap(
            lower_lid,
            upper_lid,
            axes="x",
            min_overlap=0.25,
            name="sliding lids can pass over one another longitudinally",
        )

    ctx.check(
        "lower lid slides along the freezer length",
        lower_rest is not None
        and lower_open is not None
        and lower_open[0] > lower_rest[0] + 0.40
        and abs(lower_open[1] - lower_rest[1]) < 1e-6,
        details=f"rest={lower_rest}, open={lower_open}",
    )
    ctx.check(
        "upper lid slides along the freezer length",
        upper_rest is not None
        and upper_open is not None
        and upper_open[0] < upper_rest[0] - 0.40
        and abs(upper_open[1] - upper_rest[1]) < 1e-6,
        details=f"rest={upper_rest}, open={upper_open}",
    )

    for i in range(4):
        caster = object_model.get_part(f"caster_{i}")
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.expect_within(
            wheel,
            caster,
            axes="y",
            margin=0.0,
            name=f"wheel_{i} sits between its caster fork cheeks",
        )
        ctx.expect_origin_distance(
            caster,
            wheel,
            axes="z",
            min_dist=0.13,
            max_dist=0.15,
            name=f"wheel_{i} axle is below the caster swivel",
        )

    return ctx.report()


object_model = build_object_model()
