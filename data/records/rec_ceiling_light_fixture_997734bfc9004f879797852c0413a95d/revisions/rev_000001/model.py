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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


CANOPY_RADIUS = 0.09
CANOPY_HEIGHT = 0.032
DOWNROD_RADIUS = 0.012
DOWNROD_LENGTH = 0.35
HUB_RADIUS = 0.12
HUB_TUBE = 0.013
HUB_CENTER_Z = -0.387
JOINT_Z = HUB_CENTER_Z - HUB_TUBE
PENDANT_INWARD_SHIFT = 0.014
PENDANT_DROP_LENGTHS = (0.24, 0.41, 0.30, 0.47, 0.35)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_globe_mesh():
    outer_profile = [
        (0.0, -0.078),
        (0.020, -0.076),
        (0.044, -0.064),
        (0.064, -0.038),
        (0.076, 0.000),
        (0.074, 0.038),
        (0.062, 0.067),
        (0.042, 0.087),
        (0.026, 0.096),
    ]
    inner_profile = [
        (0.0, -0.070),
        (0.015, -0.068),
        (0.038, -0.058),
        (0.056, -0.034),
        (0.067, 0.000),
        (0.065, 0.034),
        (0.055, 0.061),
        (0.037, 0.079),
        (0.018, 0.090),
    ]
    globe_geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    return _save_mesh("globe_shade", globe_geom)


def _build_pendant_cord_mesh(length: float, inward_shift: float):
    cord_end_z = -(0.024 + length)
    points = [
        (0.0, 0.0, -0.024),
        (0.0, 0.0, -0.090),
        (-0.5 * inward_shift, 0.0, -(0.14 + 0.25 * length)),
        (-inward_shift, 0.0, cord_end_z),
    ]
    return _save_mesh(
        f"pendant_cord_{int(round(length * 1000.0))}",
        tube_from_spline_points(
            points,
            radius=0.0035,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )


def _add_pendant(
    model: ArticulatedObject,
    *,
    index: int,
    angle: float,
    drop_length: float,
    metal,
    cord_material,
    glass,
    globe_mesh,
):
    pendant = model.part(f"pendant_{index + 1}")
    pendant.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=metal,
        name="holder_knuckle",
    )

    pendant.visual(
        _build_pendant_cord_mesh(drop_length, PENDANT_INWARD_SHIFT),
        material=cord_material,
        name="cord",
    )

    cord_end_z = -(0.024 + drop_length)
    pendant.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(-PENDANT_INWARD_SHIFT, 0.0, cord_end_z - 0.013)),
        material=metal,
        name="globe_neck",
    )
    pendant.visual(
        globe_mesh,
        origin=Origin(
            xyz=(-PENDANT_INWARD_SHIFT, 0.0, cord_end_z - 0.026 - 0.090),
        ),
        material=glass,
        name="globe_shade",
    )

    model.articulation(
        f"pendant_{index + 1}_hinge",
        ArticulationType.REVOLUTE,
        parent="fixture_body",
        child=pendant,
        origin=Origin(
            xyz=(HUB_RADIUS * math.cos(angle), HUB_RADIUS * math.sin(angle), JOINT_Z),
            rpy=(0.0, 0.0, angle),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.4,
            lower=-0.75,
            upper=0.75,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pendant_cluster_ceiling_fixture")

    aged_brass = model.material("aged_brass", rgba=(0.70, 0.58, 0.32, 1.0))
    cord_black = model.material("cord_black", rgba=(0.08, 0.08, 0.09, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.93, 0.94, 0.96, 0.72))

    globe_mesh = _build_globe_mesh()
    hub_ring_mesh = _save_mesh(
        "hub_ring",
        TorusGeometry(
            radius=HUB_RADIUS,
            tube=HUB_TUBE,
            radial_segments=18,
            tubular_segments=72,
        ),
    )

    fixture_body = model.part("fixture_body")
    fixture_body.visual(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT * 0.5)),
        material=aged_brass,
        name="ceiling_canopy",
    )
    fixture_body.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=aged_brass,
        name="canopy_stem",
    )
    fixture_body.visual(
        Cylinder(radius=DOWNROD_RADIUS, length=DOWNROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * DOWNROD_LENGTH)),
        material=aged_brass,
        name="downrod",
    )
    fixture_body.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.375)),
        material=aged_brass,
        name="hub_collar",
    )
    fixture_body.visual(
        hub_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, HUB_CENTER_Z)),
        material=aged_brass,
        name="hub_ring",
    )
    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        fixture_body.visual(
            Box((0.114, 0.018, 0.020)),
            origin=Origin(
                xyz=(0.057 * math.cos(angle), 0.057 * math.sin(angle), HUB_CENTER_Z),
                rpy=(0.0, 0.0, angle),
            ),
            material=aged_brass,
            name=f"hub_spoke_{index + 1}",
        )
        fixture_body.visual(
            Cylinder(radius=0.0105, length=0.020),
            origin=Origin(
                xyz=(HUB_RADIUS * math.cos(angle), HUB_RADIUS * math.sin(angle), JOINT_Z + 0.010),
            ),
            material=aged_brass,
            name=f"hub_lug_{index + 1}",
        )

    for index in range(5):
        _add_pendant(
            model,
            index=index,
            angle=(2.0 * math.pi * index) / 5.0,
            drop_length=PENDANT_DROP_LENGTHS[index],
            metal=aged_brass,
            cord_material=cord_black,
            glass=frosted_glass,
            globe_mesh=globe_mesh,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixture_body = object_model.get_part("fixture_body")
    pendants = [object_model.get_part(f"pendant_{index}") for index in range(1, 6)]
    hinges = [object_model.get_articulation(f"pendant_{index}_hinge") for index in range(1, 6)]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for pendant in pendants:
        lug_name = f"hub_lug_{pendant.name.split('_')[-1]}"
        ctx.expect_contact(
            pendant,
            fixture_body,
            elem_a="holder_knuckle",
            elem_b=lug_name,
            contact_tol=1e-6,
            name=f"{pendant.name}_mounted_to_hub",
        )
        ctx.expect_gap(
            fixture_body,
            pendant,
            axis="z",
            positive_elem="hub_ring",
            negative_elem="globe_shade",
            min_gap=0.15,
            name=f"{pendant.name}_globe_below_hub",
        )

    for index, hinge in enumerate(hinges):
        angle = (2.0 * math.pi * index) / 5.0
        ctx.check(
            f"{hinge.name}_local_axis_orientation",
            abs(hinge.axis[0]) < 1e-6
            and abs(hinge.axis[1] - 1.0) < 1e-6
            and abs(hinge.axis[2]) < 1e-6,
            details=f"expected local hinge axis (0, 1, 0), got {hinge.axis}",
        )
        ctx.check(
            f"{hinge.name}_azimuth_alignment",
            abs(hinge.origin.rpy[2] - angle) < 1e-6,
            details=f"expected hinge yaw {angle}, got {hinge.origin.rpy[2]}",
        )
        limits = hinge.motion_limits
        ctx.check(
            f"{hinge.name}_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower + 0.75) < 1e-6
            and abs(limits.upper - 0.75) < 1e-6,
            details="Each pendant hinge should have a realistic ±0.75 rad swivel range.",
        )

    pendant_1 = pendants[0]
    hinge_1 = hinges[0]
    rest_aabb = ctx.part_element_world_aabb(pendant_1, elem="globe_shade")
    ctx.check("pendant_1_globe_exists", rest_aabb is not None, details="Missing globe shade.")
    if rest_aabb is not None:
        with ctx.pose({hinge_1: 0.55}):
            swung_aabb = ctx.part_element_world_aabb(pendant_1, elem="globe_shade")
            ctx.check(
                "pendant_1_hinge_swings_globe_upward",
                swung_aabb is not None and swung_aabb[1][2] > rest_aabb[1][2] + 0.08,
                details="Opening the hinge should raise the globe noticeably relative to rest.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
