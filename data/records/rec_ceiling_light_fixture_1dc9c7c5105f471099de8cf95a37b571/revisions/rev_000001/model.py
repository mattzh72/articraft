from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _radial_arm_mesh(
    *,
    points: list[tuple[float, float, float]],
    count: int,
    radius: float,
    name: str,
) -> object:
    base = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    merged = base.clone()
    for idx in range(1, count):
        merged.merge(base.clone().rotate_z(2.0 * math.pi * idx / count))
    return mesh_from_geometry(merged, name)


def _add_lamp_cluster(
    part,
    *,
    tier_prefix: str,
    count: int,
    arm_radius: float,
    end_radius: float,
    end_z: float,
    cup_radius: float,
    bobeche_radius: float,
    brass,
    bulb,
    crystal,
) -> None:
    for idx in range(count):
        angle = 2.0 * math.pi * idx / count
        x, y = _polar_xy(end_radius, angle)
        stem_radius = max(arm_radius * 0.24, 0.0018)

        part.visual(
            Cylinder(radius=bobeche_radius, length=0.006),
            origin=Origin(xyz=(x, y, end_z + 0.003)),
            material=brass,
            name=f"elem_{tier_prefix}_bobeche_{idx}",
        )
        part.visual(
            Cylinder(radius=cup_radius, length=0.040),
            origin=Origin(xyz=(x, y, end_z + 0.026)),
            material=brass,
            name=f"elem_{tier_prefix}_cup_{idx}",
        )
        part.visual(
            Sphere(radius=0.0165),
            origin=Origin(xyz=(x, y, end_z + 0.055)),
            material=bulb,
            name=f"elem_{tier_prefix}_bulb_{idx}",
        )
        part.visual(
            Cylinder(radius=stem_radius, length=0.026),
            origin=Origin(xyz=(x, y, end_z - 0.013)),
            material=crystal,
            name=f"elem_{tier_prefix}_crystal_stem_{idx}",
        )
        part.visual(
            Sphere(radius=0.0055),
            origin=Origin(xyz=(x, y, end_z - 0.029)),
            material=crystal,
            name=f"elem_{tier_prefix}_crystal_bead_{idx}",
        )
        part.visual(
            Sphere(radius=0.0115),
            origin=Origin(xyz=(x, y, end_z - 0.044)),
            material=crystal,
            name=f"elem_{tier_prefix}_crystal_drop_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tiered_crystal_chandelier")

    antique_brass = model.material("antique_brass", rgba=(0.72, 0.61, 0.36, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.80, 0.70, 0.43, 1.0))
    crystal = model.material("crystal", rgba=(0.88, 0.94, 1.0, 0.62))
    warm_bulb = model.material("warm_bulb", rgba=(1.0, 0.96, 0.86, 0.92))

    ceiling_plate = model.part("ceiling_plate")
    ceiling_plate.visual(
        Cylinder(radius=0.110, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=antique_brass,
        name="elem_ceiling_plate",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.056, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=satin_brass,
        name="elem_canopy_knuckle",
    )

    chandelier_body = model.part("chandelier_body")
    chandelier_body.visual(
        Cylinder(radius=0.044, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=antique_brass,
        name="elem_rotation_collar",
    )
    chandelier_body.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=satin_brass,
        name="elem_upper_knuckle",
    )
    chandelier_body.visual(
        Cylinder(radius=0.012, length=0.940),
        origin=Origin(xyz=(0.0, 0.0, -0.562)),
        material=antique_brass,
        name="elem_central_rod",
    )

    upper_tier_z = -0.255
    middle_tier_z = -0.530
    lower_tier_z = -0.805

    chandelier_body.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, upper_tier_z)),
        material=satin_brass,
        name="elem_upper_tier_collar",
    )
    chandelier_body.visual(
        Cylinder(radius=0.030, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, middle_tier_z)),
        material=satin_brass,
        name="elem_middle_tier_collar",
    )
    chandelier_body.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, lower_tier_z)),
        material=satin_brass,
        name="elem_lower_tier_collar",
    )

    chandelier_body.visual(
        mesh_from_geometry(TorusGeometry(0.115, 0.014), "main_hub_ring"),
        origin=Origin(xyz=(0.0, 0.0, middle_tier_z)),
        material=antique_brass,
        name="elem_main_hub_ring",
    )

    upper_arm_end_radius = 0.255
    upper_arm_end_z = upper_tier_z + 0.050
    chandelier_body.visual(
        _radial_arm_mesh(
            points=[
                (0.022, 0.0, upper_tier_z),
                (0.095, 0.0, upper_tier_z + 0.014),
                (0.188, 0.0, upper_tier_z + 0.032),
                (upper_arm_end_radius, 0.0, upper_arm_end_z),
            ],
            count=4,
            radius=0.0085,
            name="upper_tier_arms",
        ),
        material=antique_brass,
        name="elem_upper_tier_arms",
    )

    chandelier_body.visual(
        _radial_arm_mesh(
            points=[
                (0.028, 0.0, middle_tier_z),
                (0.064, 0.0, middle_tier_z),
                (0.106, 0.0, middle_tier_z),
            ],
            count=6,
            radius=0.0055,
            name="main_hub_spokes",
        ),
        material=satin_brass,
        name="elem_main_hub_spokes",
    )

    middle_arm_end_radius = 0.365
    middle_arm_end_z = middle_tier_z + 0.055
    chandelier_body.visual(
        _radial_arm_mesh(
            points=[
                (0.122, 0.0, middle_tier_z),
                (0.220, 0.0, middle_tier_z + 0.016),
                (0.310, 0.0, middle_tier_z + 0.039),
                (middle_arm_end_radius, 0.0, middle_arm_end_z),
            ],
            count=6,
            radius=0.0095,
            name="middle_tier_arms",
        ),
        material=antique_brass,
        name="elem_middle_tier_arms",
    )

    lower_arm_end_radius = 0.485
    lower_arm_end_z = lower_tier_z + 0.062
    chandelier_body.visual(
        _radial_arm_mesh(
            points=[
                (0.024, 0.0, lower_tier_z),
                (0.165, 0.0, lower_tier_z + 0.018),
                (0.335, 0.0, lower_tier_z + 0.044),
                (lower_arm_end_radius, 0.0, lower_arm_end_z),
            ],
            count=8,
            radius=0.0105,
            name="lower_tier_arms",
        ),
        material=antique_brass,
        name="elem_lower_tier_arms",
    )

    _add_lamp_cluster(
        chandelier_body,
        tier_prefix="upper",
        count=4,
        arm_radius=0.0085,
        end_radius=upper_arm_end_radius,
        end_z=upper_arm_end_z,
        cup_radius=0.013,
        bobeche_radius=0.026,
        brass=satin_brass,
        bulb=warm_bulb,
        crystal=crystal,
    )
    _add_lamp_cluster(
        chandelier_body,
        tier_prefix="middle",
        count=6,
        arm_radius=0.0095,
        end_radius=middle_arm_end_radius,
        end_z=middle_arm_end_z,
        cup_radius=0.014,
        bobeche_radius=0.028,
        brass=satin_brass,
        bulb=warm_bulb,
        crystal=crystal,
    )
    _add_lamp_cluster(
        chandelier_body,
        tier_prefix="lower",
        count=8,
        arm_radius=0.0105,
        end_radius=lower_arm_end_radius,
        end_z=lower_arm_end_z,
        cup_radius=0.015,
        bobeche_radius=0.030,
        brass=satin_brass,
        bulb=warm_bulb,
        crystal=crystal,
    )

    chandelier_body.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.0, 0.0, -1.034)),
        material=satin_brass,
        name="elem_bottom_finial",
    )
    chandelier_body.visual(
        Cylinder(radius=0.0026, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -1.086)),
        material=crystal,
        name="elem_central_crystal_stem",
    )
    chandelier_body.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, -1.126)),
        material=crystal,
        name="elem_central_crystal_drop",
    )

    model.articulation(
        "ceiling_to_collar",
        ArticulationType.REVOLUTE,
        parent=ceiling_plate,
        child=chandelier_body,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
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

    ceiling_plate = object_model.get_part("ceiling_plate")
    chandelier_body = object_model.get_part("chandelier_body")
    swivel = object_model.get_articulation("ceiling_to_collar")

    ceiling_shell = ceiling_plate.get_visual("elem_ceiling_plate")
    collar = chandelier_body.get_visual("elem_rotation_collar")
    upper_collar = chandelier_body.get_visual("elem_upper_tier_collar")
    main_ring = chandelier_body.get_visual("elem_main_hub_ring")
    lower_collar = chandelier_body.get_visual("elem_lower_tier_collar")
    lower_bulb_0 = chandelier_body.get_visual("elem_lower_bulb_0")

    visual_names = {visual.name for visual in chandelier_body.visuals if visual.name}
    upper_bulbs = [name for name in visual_names if name.startswith("elem_upper_bulb_")]
    middle_bulbs = [name for name in visual_names if name.startswith("elem_middle_bulb_")]
    lower_bulbs = [name for name in visual_names if name.startswith("elem_lower_bulb_")]

    ctx.check(
        "tier lamp counts match chandelier tiers",
        len(upper_bulbs) == 4 and len(middle_bulbs) == 6 and len(lower_bulbs) == 8,
        details=(
            f"upper={len(upper_bulbs)}, middle={len(middle_bulbs)}, "
            f"lower={len(lower_bulbs)}"
        ),
    )

    ctx.check(
        "rotation collar uses vertical revolute axis",
        swivel.axis == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and swivel.motion_limits.lower is not None
        and swivel.motion_limits.upper is not None
        and swivel.motion_limits.lower < 0.0
        and swivel.motion_limits.upper > 0.0,
        details=f"axis={swivel.axis}, limits={swivel.motion_limits}",
    )

    ctx.expect_contact(
        chandelier_body,
        ceiling_plate,
        elem_a=collar,
        elem_b=ceiling_shell,
        contact_tol=0.0015,
        name="rotation collar seats against the ceiling plate",
    )
    ctx.expect_overlap(
        chandelier_body,
        ceiling_plate,
        axes="xy",
        elem_a=collar,
        elem_b=ceiling_shell,
        min_overlap=0.080,
        name="ceiling plate covers the rotation collar footprint",
    )

    upper_aabb = ctx.part_element_world_aabb(chandelier_body, elem=upper_collar)
    middle_aabb = ctx.part_element_world_aabb(chandelier_body, elem=main_ring)
    lower_aabb = ctx.part_element_world_aabb(chandelier_body, elem=lower_collar)
    if upper_aabb and middle_aabb and lower_aabb:
        upper_z = (upper_aabb[0][2] + upper_aabb[1][2]) * 0.5
        middle_z = (middle_aabb[0][2] + middle_aabb[1][2]) * 0.5
        lower_z = (lower_aabb[0][2] + lower_aabb[1][2]) * 0.5
        ctx.check(
            "three arm tiers descend along the rod",
            upper_z > middle_z > lower_z,
            details=f"upper_z={upper_z}, middle_z={middle_z}, lower_z={lower_z}",
        )
    else:
        ctx.fail(
            "three arm tiers descend along the rod",
            f"aabb_missing upper={upper_aabb}, middle={middle_aabb}, lower={lower_aabb}",
        )

    rest_center = None
    rotated_center = None
    rest_aabb = ctx.part_element_world_aabb(chandelier_body, elem=lower_bulb_0)
    if rest_aabb:
        rest_center = tuple(
            (rest_aabb[0][axis] + rest_aabb[1][axis]) * 0.5 for axis in range(3)
        )
    with ctx.pose({swivel: 0.75}):
        rotated_aabb = ctx.part_element_world_aabb(chandelier_body, elem=lower_bulb_0)
        if rotated_aabb:
            rotated_center = tuple(
                (rotated_aabb[0][axis] + rotated_aabb[1][axis]) * 0.5
                for axis in range(3)
            )

    ctx.check(
        "swivel rotates the chandelier about the vertical axis",
        rest_center is not None
        and rotated_center is not None
        and rotated_center[1] > rest_center[1] + 0.22
        and rotated_center[0] < rest_center[0] - 0.10
        and abs(rotated_center[2] - rest_center[2]) < 0.002,
        details=f"rest_center={rest_center}, rotated_center={rotated_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
