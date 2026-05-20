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
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_loop(radius: float, z: float, *, segments: int = 20) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
            z,
        )
        for index in range(segments)
    ]


def _tapered_leg_mesh(
    *,
    length: float,
    top_radius: float,
    bottom_radius: float,
    name: str,
) -> object:
    geom = LoftGeometry(
        [
            _circle_loop(top_radius, 0.0, segments=22),
            _circle_loop(bottom_radius, -length, segments=22),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="midcentury_tripod_floor_lamp")

    walnut = model.material("walnut", rgba=(0.46, 0.31, 0.19, 1.0))
    brass = model.material("satin_brass", rgba=(0.74, 0.62, 0.36, 1.0))
    linen = model.material("linen", rgba=(0.92, 0.89, 0.80, 0.97))
    seam = model.material("shade_seam", rgba=(0.82, 0.78, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.18, 0.16, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.12, 0.10, 1.0))

    ring_radius = 0.082
    ring_tube_radius = 0.012
    hinge_radius = 0.092
    hinge_z = -0.020
    leg_angle = math.radians(11.0)
    leg_length = 1.460
    leg_top_radius = 0.018
    leg_bottom_radius = 0.012
    shade_radius = 0.230
    shade_wall = 0.004
    shade_height = 0.300
    shade_top_z = -0.068
    shade_bottom_z = shade_top_z - shade_height

    top_ring = model.part("top_ring_assembly")
    top_ring.visual(
        mesh_from_geometry(
            TorusGeometry(radius=ring_radius, tube=ring_tube_radius, radial_segments=20, tubular_segments=40),
            "top_ring_loop",
        ),
        material=brass,
        name="ring_loop",
    )
    top_ring.visual(
        Cylinder(radius=0.028, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass,
        name="center_hub",
    )
    top_ring.visual(
        Cylinder(radius=0.017, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark_metal,
        name="shade_swivel_socket",
    )
    for strut_index in range(3):
        angle = (2.0 * math.pi / 3.0) * strut_index
        strut_length = ring_radius - 0.010
        radial_mid = (0.028 + ring_radius - 0.018) * 0.5
        top_ring.visual(
            Box((strut_length, 0.012, 0.010)),
            origin=Origin(
                xyz=(radial_mid * math.cos(angle), radial_mid * math.sin(angle), -0.004),
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"spider_strut_{strut_index}",
        )

    leg_mesh = _tapered_leg_mesh(
        length=leg_length,
        top_radius=leg_top_radius,
        bottom_radius=leg_bottom_radius,
        name="tripod_leg_taper",
    )

    for leg_index in range(3):
        angle = (2.0 * math.pi / 3.0) * leg_index
        radial = (math.cos(angle), math.sin(angle))
        tangent = (-math.sin(angle), math.cos(angle))

        for side_name, offset in (("left", -0.013), ("right", 0.013)):
            top_ring.visual(
                Box((0.036, 0.010, 0.028)),
                origin=Origin(
                    xyz=(
                        hinge_radius * radial[0] + offset * tangent[0],
                        hinge_radius * radial[1] + offset * tangent[1],
                        hinge_z,
                    ),
                    rpy=(0.0, 0.0, angle),
                ),
                material=brass,
                name=f"leg_{leg_index}_{side_name}_ear",
            )

        leg = model.part(f"leg_{leg_index}")
        leg.visual(
            Cylinder(radius=0.0085, length=0.016),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.024, 0.012, 0.044)),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=brass,
            name="hinge_cheek",
        )
        leg.visual(
            leg_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=walnut,
            name="leg_shaft",
        )
        leg.visual(
            Cylinder(radius=0.015, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, -0.070)),
            material=brass,
            name="upper_ferrule",
        )
        leg.visual(
            Cylinder(radius=0.013, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, -(leg_length + 0.020))),
            material=brass,
            name="foot_cap",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -(leg_length + 0.044))),
            material=rubber,
            name="foot_tip",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.060, 0.060, leg_length + 0.100)),
            mass=0.85,
            origin=Origin(xyz=(0.0, 0.0, -0.72)),
        )
        model.articulation(
            f"top_ring_to_leg_{leg_index}",
            ArticulationType.REVOLUTE,
            parent=top_ring,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * radial[0], hinge_radius * radial[1], hinge_z),
                rpy=(0.0, -leg_angle, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.2,
                lower=-0.10,
                upper=0.12,
            ),
        )

    outer_profile = [
        (shade_radius, shade_top_z),
        (shade_radius, (shade_top_z + shade_bottom_z) * 0.5),
        (shade_radius, shade_bottom_z),
    ]
    inner_profile = [
        (shade_radius - shade_wall, shade_top_z),
        (shade_radius - shade_wall, (shade_top_z + shade_bottom_z) * 0.5),
        (shade_radius - shade_wall, shade_bottom_z),
    ]
    shade_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "drum_shade_shell",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.015, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=dark_metal,
        name="swivel_collar",
    )
    shade.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, shade_top_z + 0.008)),
        material=dark_metal,
        name="spider_plate",
    )
    for spoke_index in range(3):
        angle = (2.0 * math.pi / 3.0) * spoke_index
        spoke_length = shade_radius - 0.028
        spoke_center = 0.5 * (0.028 + shade_radius)
        shade.visual(
            Box((spoke_length, 0.010, 0.004)),
            origin=Origin(
                xyz=(spoke_center * math.cos(angle), spoke_center * math.sin(angle), shade_top_z + 0.004),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"top_spoke_{spoke_index}",
        )
    shade.visual(
        mesh_from_geometry(
            TorusGeometry(radius=shade_radius - 0.002, tube=0.004, radial_segments=16, tubular_segments=40),
            "shade_top_rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, shade_top_z)),
        material=dark_metal,
        name="top_rim",
    )
    shade.visual(
        mesh_from_geometry(
            TorusGeometry(radius=shade_radius - 0.002, tube=0.004, radial_segments=16, tubular_segments=40),
            "shade_bottom_rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, shade_bottom_z)),
        material=dark_metal,
        name="bottom_rim",
    )
    shade.visual(
        shade_shell_mesh,
        material=linen,
        name="shade_shell",
    )
    shade.visual(
        Box((0.008, 0.016, shade_height - 0.018)),
        origin=Origin(
            xyz=(shade_radius - 0.001, 0.0, (shade_top_z + shade_bottom_z) * 0.5),
        ),
        material=seam,
        name="shade_seam",
    )
    shade.inertial = Inertial.from_geometry(
        Box((shade_radius * 2.0, shade_radius * 2.0, 0.420)),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
    )
    model.articulation(
        "top_ring_to_shade",
        ArticulationType.REVOLUTE,
        parent=top_ring,
        child=shade,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-1.25,
            upper=1.25,
        ),
    )

    top_ring.inertial = Inertial.from_geometry(
        Box((0.260, 0.260, 0.090)),
        mass=1.6,
        origin=Origin(),
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

    top_ring = object_model.get_part("top_ring_assembly")
    shade = object_model.get_part("shade")
    leg_parts = [object_model.get_part(f"leg_{index}") for index in range(3)]
    leg_joints = [object_model.get_articulation(f"top_ring_to_leg_{index}") for index in range(3)]
    shade_joint = object_model.get_articulation("top_ring_to_shade")

    ctx.check("lamp has three articulated legs", len(leg_parts) == 3 and len(leg_joints) == 3)
    ctx.expect_origin_distance(
        shade,
        top_ring,
        axes="xy",
        max_dist=0.001,
        name="shade swivel stays centered over the ring",
    )

    rest_foot_data: list[tuple[float, float]] = []
    for index, leg in enumerate(leg_parts):
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_tip")
        if foot_aabb is None:
            ctx.fail(f"leg {index} foot tip exists", "No AABB resolved for foot_tip.")
            continue
        foot_center = tuple((foot_aabb[0][axis] + foot_aabb[1][axis]) * 0.5 for axis in range(3))
        radial = math.hypot(foot_center[0], foot_center[1])
        rest_foot_data.append((radial, foot_center[2]))
        ctx.check(
            f"leg {index} rests near the floor plane",
            -1.52 <= foot_center[2] <= -1.40,
            details=f"foot_center={foot_center}",
        )
        ctx.check(
            f"leg {index} is visibly splayed",
            radial >= 0.34,
            details=f"radial_distance={radial:.4f}",
        )

    if len(rest_foot_data) == 3:
        foot_heights = [height for _, height in rest_foot_data]
        ctx.check(
            "all three feet share a common floor contact height",
            max(foot_heights) - min(foot_heights) <= 0.01,
            details=f"foot_heights={foot_heights}",
        )

    for index, (leg, joint) in enumerate(zip(leg_parts, leg_joints)):
        rest_aabb = ctx.part_element_world_aabb(leg, elem="foot_tip")
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper if upper is not None else 0.0}):
            folded_aabb = ctx.part_element_world_aabb(leg, elem="foot_tip")
        if rest_aabb is None or folded_aabb is None:
            ctx.fail(
                f"leg {index} folding pose resolves",
                f"rest_aabb={rest_aabb}, folded_aabb={folded_aabb}",
            )
            continue
        rest_center = tuple((rest_aabb[0][axis] + rest_aabb[1][axis]) * 0.5 for axis in range(3))
        folded_center = tuple((folded_aabb[0][axis] + folded_aabb[1][axis]) * 0.5 for axis in range(3))
        rest_radial = math.hypot(rest_center[0], rest_center[1])
        folded_radial = math.hypot(folded_center[0], folded_center[1])
        ctx.check(
            f"leg {index} folds inward toward the center",
            folded_radial <= rest_radial - 0.10,
            details=f"rest_center={rest_center}, folded_center={folded_center}",
        )

    seam_rest = ctx.part_element_world_aabb(shade, elem="shade_seam")
    with ctx.pose({shade_joint: 0.90}):
        seam_rotated = ctx.part_element_world_aabb(shade, elem="shade_seam")
    if seam_rest is None or seam_rotated is None:
        ctx.fail("shade seam pose samples resolve", f"seam_rest={seam_rest}, seam_rotated={seam_rotated}")
    else:
        seam_rest_center = tuple((seam_rest[0][axis] + seam_rest[1][axis]) * 0.5 for axis in range(3))
        seam_rot_center = tuple((seam_rotated[0][axis] + seam_rotated[1][axis]) * 0.5 for axis in range(3))
        rest_radius = math.hypot(seam_rest_center[0], seam_rest_center[1])
        rotated_radius = math.hypot(seam_rot_center[0], seam_rot_center[1])
        seam_shift = math.dist(seam_rest_center[:2], seam_rot_center[:2])
        ctx.check(
            "shade rotates about the ring center",
            seam_shift >= 0.12 and abs(rest_radius - rotated_radius) <= 0.02,
            details=(
                f"seam_rest_center={seam_rest_center}, seam_rot_center={seam_rot_center}, "
                f"rest_radius={rest_radius:.4f}, rotated_radius={rotated_radius:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
