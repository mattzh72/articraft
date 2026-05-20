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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_top_lazy_susan")

    brushed_steel = model.material("brushed_steel", rgba=(0.71, 0.73, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.25, 0.27, 0.29, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.78, 0.88, 0.94, 0.30))

    base_outer_radius = 0.170
    base_inner_radius = 0.118
    base_ring_height = 0.024
    hub_radius = 0.042
    hub_barrel_height = 0.014
    glass_radius = 0.225
    glass_thickness = 0.012
    joint_z = base_ring_height + hub_barrel_height

    base_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(base_outer_radius),
            [_circle_profile(base_inner_radius)],
            height=base_ring_height,
            center=True,
        ),
        "lazy_susan_base_ring",
    )

    base = model.part("base_assembly")
    base.visual(
        base_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, base_ring_height * 0.5)),
        material=dark_metal,
        name="base_ring",
    )
    spoke_length = base_inner_radius - hub_radius
    spoke_offset = hub_radius + (spoke_length * 0.5)
    for axis_name, xyz, size in (
        ("x_pos", (spoke_offset, 0.0, 0.010), (spoke_length, 0.022, 0.010)),
        ("x_neg", (-spoke_offset, 0.0, 0.010), (spoke_length, 0.022, 0.010)),
        ("y_pos", (0.0, spoke_offset, 0.010), (0.022, spoke_length, 0.010)),
        ("y_neg", (0.0, -spoke_offset, 0.010), (0.022, spoke_length, 0.010)),
    ):
        base.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=brushed_steel,
            name=f"support_spoke_{axis_name}",
        )
    base.visual(
        Cylinder(radius=hub_radius, length=base_ring_height),
        origin=Origin(xyz=(0.0, 0.0, base_ring_height * 0.5)),
        material=brushed_steel,
        name="hub_core",
    )
    base.visual(
        Cylinder(radius=hub_radius, length=hub_barrel_height),
        origin=Origin(xyz=(0.0, 0.0, base_ring_height + hub_barrel_height * 0.5)),
        material=brushed_steel,
        name="hub_barrel",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_outer_radius * 2.0, base_outer_radius * 2.0, joint_z)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, joint_z * 0.5)),
    )

    upper_disc = model.part("upper_disc")
    upper_disc.visual(
        Cylinder(radius=glass_radius, length=glass_thickness),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=smoked_glass,
        name="glass_plate",
    )
    upper_disc.visual(
        Cylinder(radius=hub_radius, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=brushed_steel,
        name="lower_bearing_hub",
    )
    upper_disc.visual(
        Cylinder(radius=0.032, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0195)),
        material=brushed_steel,
        name="upper_hub_cap",
    )
    upper_disc.inertial = Inertial.from_geometry(
        Box((glass_radius * 2.0, glass_radius * 2.0, 0.020)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    model.articulation(
        "base_to_upper_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=upper_disc,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=4.0,
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

    base = object_model.get_part("base_assembly")
    upper_disc = object_model.get_part("upper_disc")
    spin = object_model.get_articulation("base_to_upper_disc")

    base_ring = base.get_visual("base_ring")
    hub_barrel = base.get_visual("hub_barrel")
    glass_plate = upper_disc.get_visual("glass_plate")
    lower_bearing_hub = upper_disc.get_visual("lower_bearing_hub")

    limits = spin.motion_limits
    ctx.check(
        "lazy susan uses continuous vertical spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={spin.articulation_type}, axis={spin.axis}, "
            f"limits={(None if limits is None else (limits.lower, limits.upper))}"
        ),
    )

    with ctx.pose({spin: 0.0}):
        ctx.expect_gap(
            upper_disc,
            base,
            axis="z",
            positive_elem=glass_plate,
            negative_elem=base_ring,
            min_gap=0.017,
            max_gap=0.020,
            name="glass plate sits clearly above the base ring",
        )
        ctx.expect_contact(
            upper_disc,
            base,
            elem_a=lower_bearing_hub,
            elem_b=hub_barrel,
            name="bearing hub halves meet at the spin axis",
        )
        ctx.expect_origin_distance(
            upper_disc,
            base,
            axes="xy",
            max_dist=1e-6,
            name="upper disc remains centered over the base",
        )
        ctx.expect_overlap(
            upper_disc,
            base,
            axes="xy",
            elem_a=glass_plate,
            elem_b=base_ring,
            min_overlap=0.30,
            name="glass plate fully covers the base footprint",
        )

    rest_pos = ctx.part_world_position(upper_disc)
    with ctx.pose({spin: 1.8}):
        turned_pos = ctx.part_world_position(upper_disc)
        ctx.expect_contact(
            upper_disc,
            base,
            elem_a=lower_bearing_hub,
            elem_b=hub_barrel,
            name="hub contact is preserved while rotated",
        )
        ctx.expect_gap(
            upper_disc,
            base,
            axis="z",
            positive_elem=glass_plate,
            negative_elem=base_ring,
            min_gap=0.017,
            max_gap=0.020,
            name="glass plate keeps its lift while rotated",
        )

    ctx.check(
        "upper disc spins in place about the center",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
