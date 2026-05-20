from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh_from_profile(profile: list[tuple[float, float]], height: float, name: str, *, z0: float = 0.0):
    geom = ExtrudeGeometry.from_z0(profile, height, cap=True, closed=True)
    if z0:
        geom.translate(0.0, 0.0, z0)
    return mesh_from_geometry(geom, name)


def _mesh_from_ring(
    outer: list[tuple[float, float]],
    inner: list[tuple[float, float]],
    height: float,
    name: str,
    *,
    z0: float = 0.0,
):
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        height,
        cap=True,
        center=False,
        closed=True,
    )
    if z0:
        geom.translate(0.0, 0.0, z0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_display_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.86, 0.87, 0.84, 1.0))
    liner_white = model.material("liner_white", rgba=(0.94, 0.95, 0.96, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.66, 0.80, 0.88, 0.45))

    outer_profile = [
        (-0.60, -0.34),
        (0.42, -0.34),
        (0.60, 0.34),
        (-0.42, 0.34),
    ]
    opening_profile = [
        (-0.50, -0.26),
        (0.36, -0.26),
        (0.50, 0.26),
        (-0.36, 0.26),
    ]
    top_trim_outer = [
        (-0.62, -0.36),
        (0.44, -0.36),
        (0.62, 0.36),
        (-0.44, 0.36),
    ]

    cabinet = model.part("cabinet")
    cabinet.visual(
        _mesh_from_ring(outer_profile, opening_profile, 0.68, "cabinet_shell", z0=0.10),
        material=cabinet_white,
        name="shell",
    )
    cabinet.visual(
        _mesh_from_profile(opening_profile, 0.05, "cabinet_floor", z0=0.10),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        _mesh_from_ring(top_trim_outer, opening_profile, 0.02, "cabinet_top_trim", z0=0.78),
        material=dark_trim,
        name="top_trim",
    )

    rail_length = 0.92
    rail_depth = 0.028
    rail_thickness = 0.006
    front_rail_x = -0.07
    back_rail_x = 0.07
    cabinet.visual(
        Box((rail_length, rail_depth, rail_thickness)),
        origin=Origin(xyz=(front_rail_x, -0.246, 0.797)),
        material=aluminum,
        name="rail_front_low",
    )
    cabinet.visual(
        Box((rail_length, rail_depth, rail_thickness)),
        origin=Origin(xyz=(back_rail_x, 0.246, 0.797)),
        material=aluminum,
        name="rail_back_low",
    )
    cabinet.visual(
        Box((rail_length, rail_depth, rail_thickness)),
        origin=Origin(xyz=(front_rail_x, -0.246, 0.811)),
        material=aluminum,
        name="rail_front_high",
    )
    cabinet.visual(
        Box((rail_length, rail_depth, rail_thickness)),
        origin=Origin(xyz=(back_rail_x, 0.246, 0.811)),
        material=aluminum,
        name="rail_back_high",
    )
    cabinet.visual(
        Box((0.032, rail_depth, 0.008)),
        origin=Origin(xyz=(0.16, -0.246, 0.804)),
        material=aluminum,
        name="rail_front_high_support_left",
    )
    cabinet.visual(
        Box((0.032, rail_depth, 0.008)),
        origin=Origin(xyz=(0.36, -0.246, 0.804)),
        material=aluminum,
        name="rail_front_high_support_right",
    )
    cabinet.visual(
        Box((0.016, rail_depth, 0.008)),
        origin=Origin(xyz=(0.24, 0.246, 0.804)),
        material=aluminum,
        name="rail_back_high_support_left",
    )
    cabinet.visual(
        Box((0.032, rail_depth, 0.008)),
        origin=Origin(xyz=(0.50, 0.246, 0.804)),
        material=aluminum,
        name="rail_back_high_support_right",
    )

    for mount_name, mount_xyz in (
        ("front_left_mount", (-0.47, -0.25, 0.096)),
        ("front_right_mount", (0.33, -0.25, 0.096)),
        ("rear_right_mount", (0.47, 0.25, 0.096)),
        ("rear_left_mount", (-0.33, 0.25, 0.096)),
    ):
        cabinet.visual(
            Box((0.060, 0.060, 0.008)),
            origin=Origin(xyz=mount_xyz),
            material=dark_trim,
            name=mount_name,
        )

    lid_outer_local = [
        (-0.31, -0.26),
        (0.17, -0.26),
        (0.31, 0.26),
        (-0.17, 0.26),
    ]
    lid_hole_local = [
        (-0.27, -0.22),
        (0.14, -0.22),
        (0.27, 0.22),
        (-0.14, 0.22),
    ]
    lid_glass_local = [
        (-0.275, -0.225),
        (0.145, -0.225),
        (0.275, 0.225),
        (-0.145, 0.225),
    ]

    left_lid = model.part("left_lid")
    left_lid.visual(
        _mesh_from_ring(lid_outer_local, lid_hole_local, 0.010, "left_lid_frame"),
        material=aluminum,
        name="frame",
    )
    left_lid.visual(
        _mesh_from_profile(lid_glass_local, 0.006, "left_lid_glass", z0=0.002),
        material=glass_tint,
        name="glass",
    )

    right_lid = model.part("right_lid")
    right_lid.visual(
        _mesh_from_ring(lid_outer_local, lid_hole_local, 0.010, "right_lid_frame"),
        material=aluminum,
        name="frame",
    )
    right_lid.visual(
        _mesh_from_profile(lid_glass_local, 0.006, "right_lid_glass", z0=0.002),
        material=glass_tint,
        name="glass",
    )

    model.articulation(
        "cabinet_to_left_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=left_lid,
        origin=Origin(xyz=(-0.19, 0.0, 0.800)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.38, effort=50.0, velocity=0.4),
    )
    model.articulation(
        "cabinet_to_right_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=right_lid,
        origin=Origin(xyz=(0.19, 0.0, 0.814)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.38, effort=50.0, velocity=0.4),
    )

    wheel_black = model.material("wheel_black", rgba=(0.09, 0.09, 0.10, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.55, 0.57, 0.60, 1.0))

    caster_specs = (
        ("front_left", (-0.47, -0.25, 0.092), "front_left_mount"),
        ("front_right", (0.33, -0.25, 0.092), "front_right_mount"),
        ("rear_right", (0.47, 0.25, 0.092), "rear_right_mount"),
        ("rear_left", (-0.33, 0.25, 0.092), "rear_left_mount"),
    )
    for caster_name, caster_xyz, mount_name in caster_specs:
        caster = model.part(f"{caster_name}_caster")
        caster.visual(
            Cylinder(radius=0.028, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=dark_trim,
            name="swivel_plate",
        )
        caster.visual(
            Cylinder(radius=0.008, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=aluminum,
            name="stem",
        )
        caster.visual(
            Box((0.032, 0.028, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.032)),
            material=dark_trim,
            name="fork_bridge",
        )
        caster.visual(
            Box((0.028, 0.005, 0.030)),
            origin=Origin(xyz=(0.0, -0.0135, -0.051)),
            material=aluminum,
            name="fork_left",
        )
        caster.visual(
            Box((0.028, 0.005, 0.030)),
            origin=Origin(xyz=(0.0, 0.0135, -0.051)),
            material=aluminum,
            name="fork_right",
        )
        wheel = model.part(f"{caster_name}_wheel")
        wheel.visual(
            Cylinder(radius=0.028, length=0.022),
            origin=Origin(rpy=(1.5708, 0.0, 0.0)),
            material=wheel_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.020, length=0.024),
            origin=Origin(rpy=(1.5708, 0.0, 0.0)),
            material=hub_gray,
            name="hub",
        )

        model.articulation(
            f"cabinet_to_{caster_name}_caster",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=caster,
            origin=Origin(xyz=caster_xyz),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=8.0),
        )
        model.articulation(
            f"{caster_name}_caster_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.064)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
            meta={"mount_visual": mount_name},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    cabinet = object_model.get_part("cabinet")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    left_slide = object_model.get_articulation("cabinet_to_left_lid")
    right_slide = object_model.get_articulation("cabinet_to_right_lid")

    ctx.expect_contact(
        left_lid,
        cabinet,
        elem_a="frame",
        elem_b="rail_front_low",
        name="left lid sits on lower guide rail",
    )
    ctx.expect_contact(
        right_lid,
        cabinet,
        elem_a="frame",
        elem_b="rail_front_high",
        name="right lid sits on upper guide rail",
    )
    ctx.expect_overlap(
        left_lid,
        right_lid,
        axes="xy",
        min_overlap=0.08,
        name="closed lids overlap in plan to cover the opening",
    )
    ctx.expect_gap(
        right_lid,
        left_lid,
        axis="z",
        min_gap=0.003,
        max_gap=0.020,
        positive_elem="frame",
        negative_elem="frame",
        name="offset lid tracks keep the glass sliders vertically separated",
    )

    left_rest = ctx.part_world_position(left_lid)
    right_rest = ctx.part_world_position(right_lid)
    with ctx.pose({left_slide: 0.38, right_slide: 0.38}):
        left_open = ctx.part_world_position(left_lid)
        right_open = ctx.part_world_position(right_lid)

    ctx.check(
        "left lid slides right when opened",
        left_rest is not None and left_open is not None and left_open[0] > left_rest[0] + 0.30,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right lid slides left when opened",
        right_rest is not None and right_open is not None and right_open[0] < right_rest[0] - 0.30,
        details=f"rest={right_rest}, open={right_open}",
    )

    for caster_name in ("front_left", "front_right", "rear_right", "rear_left"):
        caster = object_model.get_part(f"{caster_name}_caster")
        wheel = object_model.get_part(f"{caster_name}_wheel")
        swivel = object_model.get_articulation(f"cabinet_to_{caster_name}_caster")
        wheel_spin = object_model.get_articulation(f"{caster_name}_caster_to_wheel")

        ctx.expect_contact(
            caster,
            cabinet,
            elem_a="swivel_plate",
            elem_b=f"{caster_name}_mount",
            name=f"{caster_name} caster is mounted to cabinet corner pad",
        )
        ctx.expect_contact(
            wheel,
            caster,
            elem_a="tire",
            elem_b="fork_left",
            name=f"{caster_name} wheel is captured by one fork leg",
        )
        ctx.check(
            f"{caster_name} caster swivel axis is vertical",
            swivel.axis == (0.0, 0.0, 1.0),
            details=f"axis={swivel.axis}",
        )
        ctx.check(
            f"{caster_name} wheel spin axis is lateral",
            wheel_spin.axis == (0.0, 1.0, 0.0),
            details=f"axis={wheel_spin.axis}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
