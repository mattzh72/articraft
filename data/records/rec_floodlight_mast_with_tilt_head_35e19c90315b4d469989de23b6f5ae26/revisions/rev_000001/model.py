from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _octagon_section(across_flats: float, z: float) -> tuple[tuple[float, float, float], ...]:
    inradius = across_flats * 0.5
    circumradius = inradius / math.cos(math.pi / 8.0)
    return tuple(
        (
            circumradius * math.cos(math.pi / 8.0 + index * math.pi / 4.0),
            circumradius * math.sin(math.pi / 8.0 + index * math.pi / 4.0),
            z,
        )
        for index in range(8)
    )


def _rounded_rect_section(
    x: float,
    *,
    center_z: float,
    width_y: float,
    height_z: float,
    radius: float,
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (x, y, center_z + z)
        for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=6)
    )


def _build_pole_shaft_mesh():
    return section_loft(
        [
            _octagon_section(0.22, 0.03),
            _octagon_section(0.18, 4.30),
            _octagon_section(0.16, 8.33),
        ]
    )


def _build_flood_head_shell_mesh():
    return section_loft(
        [
            _rounded_rect_section(
                0.06,
                center_z=-0.072,
                width_y=0.17,
                height_z=0.08,
                radius=0.015,
            ),
            _rounded_rect_section(
                0.22,
                center_z=-0.102,
                width_y=0.215,
                height_z=0.13,
                radius=0.022,
            ),
            _rounded_rect_section(
                0.39,
                center_z=-0.112,
                width_y=0.27,
                height_z=0.17,
                radius=0.028,
            ),
        ]
    )


def _add_anchor_bolts(part, material) -> None:
    bolt_radius = 0.012
    bolt_height = 0.09
    bolt_xy = 0.11
    for x in (-bolt_xy, bolt_xy):
        for y in (-bolt_xy, bolt_xy):
            part.visual(
                Cylinder(radius=bolt_radius, length=bolt_height),
                origin=Origin(xyz=(x, y, 0.045)),
                material=material,
            )


def _add_flood_head_visuals(part, *, shell_mesh, housing_material, trim_material, lens_material) -> None:
    part.visual(
        Cylinder(radius=0.024, length=0.092),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.11, 0.07, 0.05)),
        origin=Origin(xyz=(0.055, 0.0, -0.045)),
        material=trim_material,
        name="mount_block",
    )
    part.visual(
        shell_mesh,
        material=housing_material,
        name="housing_shell",
    )
    part.visual(
        Box((0.022, 0.262, 0.172)),
        origin=Origin(xyz=(0.401, 0.0, -0.112)),
        material=trim_material,
        name="front_bezel",
    )
    part.visual(
        Box((0.006, 0.232, 0.142)),
        origin=Origin(xyz=(0.415, 0.0, -0.112)),
        material=lens_material,
        name="lens",
    )
    part.visual(
        Box((0.085, 0.08, 0.05)),
        origin=Origin(xyz=(0.01, 0.0, -0.092)),
        material=trim_material,
        name="driver_box",
    )
    part.visual(
        Box((0.015, 0.075, 0.05)),
        origin=Origin(xyz=(-0.026, 0.0, -0.092)),
        material=trim_material,
        name="heat_sink_fin_center",
    )
    part.visual(
        Box((0.015, 0.075, 0.04)),
        origin=Origin(xyz=(-0.039, 0.0, -0.062)),
        material=trim_material,
        name="heat_sink_fin_upper",
    )
    part.visual(
        Box((0.015, 0.075, 0.04)),
        origin=Origin(xyz=(-0.039, 0.0, -0.122)),
        material=trim_material,
        name="heat_sink_fin_lower",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_lot_floodlight")

    concrete = model.material("concrete", rgba=(0.70, 0.70, 0.69, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.61, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    housing_black = model.material("housing_black", rgba=(0.12, 0.13, 0.14, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.80, 0.86, 0.92, 1.0))

    pole_shaft_mesh = mesh_from_geometry(_build_pole_shaft_mesh(), "octagonal_pole_shaft")
    flood_head_shell_mesh = mesh_from_geometry(_build_flood_head_shell_mesh(), "flood_head_shell")

    base = model.part("concrete_base")
    base.visual(
        Box((0.72, 0.72, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=concrete,
        name="base_block",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 0.32)),
        mass=380.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    pole = model.part("pole_assembly")
    pole.visual(
        Box((0.34, 0.34, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="pole_flange",
    )
    _add_anchor_bolts(pole, dark_steel)
    pole.visual(
        pole_shaft_mesh,
        material=galvanized_steel,
        name="pole_shaft",
    )
    pole.visual(
        Box((0.30, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 8.39)),
        material=dark_steel,
        name="top_collar",
    )
    pole.visual(
        Box((0.64, 0.10, 0.10)),
        origin=Origin(xyz=(0.48, 0.0, 8.39)),
        material=dark_steel,
        name="right_outreach_arm",
    )
    pole.visual(
        Box((0.64, 0.10, 0.10)),
        origin=Origin(xyz=(-0.48, 0.0, 8.39)),
        material=dark_steel,
        name="left_outreach_arm",
    )

    brace_length = math.hypot(0.40, 0.15)
    brace_angle = math.atan2(0.15, 0.40)
    pole.visual(
        Box((brace_length, 0.045, 0.045)),
        origin=Origin(xyz=(0.28, 0.0, 8.255), rpy=(0.0, -brace_angle, 0.0)),
        material=dark_steel,
        name="right_brace",
    )
    pole.visual(
        Box((brace_length, 0.045, 0.045)),
        origin=Origin(xyz=(-0.28, 0.0, 8.255), rpy=(0.0, brace_angle, 0.0)),
        material=dark_steel,
        name="left_brace",
    )

    lug_size = (0.04, 0.018, 0.16)
    lug_y = 0.055
    for side_name, x_pos in (("right", 0.82), ("left", -0.82)):
        pole.visual(
            Box(lug_size),
            origin=Origin(xyz=(x_pos, lug_y, 8.32)),
            material=dark_steel,
            name=f"{side_name}_outer_lug",
        )
        pole.visual(
            Box(lug_size),
            origin=Origin(xyz=(x_pos, -lug_y, 8.32)),
            material=dark_steel,
            name=f"{side_name}_inner_lug",
        )

    pole.inertial = Inertial.from_geometry(
        Box((1.72, 0.34, 8.50)),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, 4.25)),
    )

    right_head = model.part("right_flood_head")
    _add_flood_head_visuals(
        right_head,
        shell_mesh=flood_head_shell_mesh,
        housing_material=housing_black,
        trim_material=dark_steel,
        lens_material=lens_glass,
    )
    right_head.inertial = Inertial.from_geometry(
        Box((0.45, 0.28, 0.21)),
        mass=18.0,
        origin=Origin(xyz=(0.19, 0.0, -0.10)),
    )

    left_head = model.part("left_flood_head")
    _add_flood_head_visuals(
        left_head,
        shell_mesh=flood_head_shell_mesh,
        housing_material=housing_black,
        trim_material=dark_steel,
        lens_material=lens_glass,
    )
    left_head.inertial = Inertial.from_geometry(
        Box((0.45, 0.28, 0.21)),
        mass=18.0,
        origin=Origin(xyz=(0.19, 0.0, -0.10)),
    )

    model.articulation(
        "base_to_pole",
        ArticulationType.FIXED,
        parent=base,
        child=pole,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
    )
    model.articulation(
        "pole_to_right_head",
        ArticulationType.REVOLUTE,
        parent=pole,
        child=right_head,
        origin=Origin(xyz=(0.82, 0.0, 8.32)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.9,
            lower=-0.35,
            upper=1.15,
        ),
    )
    model.articulation(
        "pole_to_left_head",
        ArticulationType.REVOLUTE,
        parent=pole,
        child=left_head,
        origin=Origin(xyz=(-0.82, 0.0, 8.32), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.9,
            lower=-0.35,
            upper=1.15,
        ),
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

    base = object_model.get_part("concrete_base")
    pole = object_model.get_part("pole_assembly")
    left_head = object_model.get_part("left_flood_head")
    right_head = object_model.get_part("right_flood_head")
    left_tilt = object_model.get_articulation("pole_to_left_head")
    right_tilt = object_model.get_articulation("pole_to_right_head")

    ctx.expect_gap(
        pole,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="pole flange seats cleanly on the concrete base",
    )
    ctx.expect_overlap(
        pole,
        base,
        axes="xy",
        min_overlap=0.30,
        name="pole assembly stays centered on the square base",
    )
    ctx.expect_origin_gap(
        right_head,
        pole,
        axis="x",
        min_gap=0.70,
        name="right flood head is carried outboard by its bracket",
    )
    ctx.expect_origin_gap(
        pole,
        left_head,
        axis="x",
        min_gap=0.70,
        name="left flood head is carried outboard by its bracket",
    )
    ctx.expect_contact(
        pole,
        right_head,
        contact_tol=5e-5,
        name="right flood head is supported at the hinge bracket",
    )
    ctx.expect_contact(
        pole,
        left_head,
        contact_tol=5e-5,
        name="left flood head is supported at the hinge bracket",
    )

    def element_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((low + high) * 0.5 for low, high in zip(mins, maxs))

    right_rest_center = element_center(right_head, "lens")
    left_rest_center = element_center(left_head, "lens")
    with ctx.pose({right_tilt: 0.85, left_tilt: 0.85}):
        right_down_center = element_center(right_head, "lens")
        left_down_center = element_center(left_head, "lens")

    ctx.check(
        "right flood head tilts nose-down",
        right_rest_center is not None
        and right_down_center is not None
        and right_down_center[2] < right_rest_center[2] - 0.10,
        details=f"rest={right_rest_center}, tilted={right_down_center}",
    )
    ctx.check(
        "left flood head tilts nose-down",
        left_rest_center is not None
        and left_down_center is not None
        and left_down_center[2] < left_rest_center[2] - 0.10,
        details=f"rest={left_rest_center}, tilted={left_down_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
