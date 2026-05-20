from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _build_focus_ring_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.0305, 0.006),
            (0.0310, 0.008),
            (0.0310, 0.028),
            (0.0305, 0.030),
        ],
        [
            (0.0268, 0.006),
            (0.0268, 0.030),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    shell.merge(
        TorusGeometry(
            radius=0.0309,
            tube=0.0010,
            radial_segments=12,
            tubular_segments=48,
        ).translate(0.0, 0.0, 0.0085)
    )
    shell.merge(
        TorusGeometry(
            radius=0.0309,
            tube=0.0010,
            radial_segments=12,
            tubular_segments=48,
        ).translate(0.0, 0.0, 0.0275)
    )
    for index in range(18):
        angle = index * math.tau / 18.0
        rib = (
            BoxGeometry((0.0034, 0.0060, 0.0190))
            .translate(0.0314, 0.0, 0.0180)
            .rotate_z(angle)
        )
        shell.merge(rib)
    return shell


def _build_body_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0205, -0.0040),
            (0.0212, -0.0010),
            (0.0228, 0.0010),
            (0.0258, 0.0080),
            (0.0262, 0.0180),
            (0.0260, 0.0260),
            (0.0256, 0.0320),
            (0.0252, 0.0420),
        ],
        [
            (0.0178, -0.0040),
            (0.0182, 0.0020),
            (0.0208, 0.0080),
            (0.0248, 0.0180),
            (0.0250, 0.0260),
            (0.0250, 0.0320),
            (0.0250, 0.0420),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )


def _build_helicoid_housing_mesh() -> MeshGeometry:
    housing = LatheGeometry.from_shell_profiles(
        [
            (0.0277, 0.028),
            (0.0285, 0.030),
            (0.0285, 0.042),
            (0.0280, 0.044),
        ],
        [
            (0.0254, 0.028),
            (0.0254, 0.044),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    housing.merge(
        TorusGeometry(
            radius=0.0280,
            tube=0.0010,
            radial_segments=12,
            tubular_segments=48,
        ).translate(0.0, 0.0, 0.0415)
    )
    return housing


def _build_mount_thread_mesh() -> MeshGeometry:
    threads = MeshGeometry()
    for z_center in (-0.0025, -0.0008, 0.0009, 0.0026):
        threads.merge(
            TorusGeometry(
                radius=0.0203,
                tube=0.0009,
                radial_segments=10,
                tubular_segments=42,
            ).translate(0.0, 0.0, z_center)
        )
    threads.merge(
        TorusGeometry(
            radius=0.0213,
            tube=0.00115,
            radial_segments=12,
            tubular_segments=48,
        ).translate(0.0, 0.0, 0.0042)
    )
    return threads


def _build_front_barrel_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0242, -0.020),
            (0.0246, -0.018),
            (0.0246, -0.004),
            (0.0249, 0.004),
            (0.0276, 0.004),
            (0.0276, 0.007),
            (0.0248, 0.007),
            (0.0248, 0.016),
            (0.0238, 0.021),
            (0.0230, 0.026),
        ],
        [
            (0.0208, -0.020),
            (0.0208, -0.004),
            (0.0204, 0.004),
            (0.0196, 0.012),
            (0.0182, 0.020),
            (0.0168, 0.026),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )


def _build_front_retainer_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.01855, -0.0020),
            (0.01840, -0.0012),
            (0.01820, 0.0000),
        ],
        [
            (0.01560, -0.0020),
            (0.01560, 0.0000),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_screw_mount_prime")

    black_enamel = model.material("black_enamel", rgba=(0.10, 0.10, 0.10, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    nickel = model.material("nickel", rgba=(0.66, 0.65, 0.62, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.16, 0.19, 0.92))

    lens_body = model.part("lens_body")
    lens_body.visual(
        _mesh("lens_body_shell", _build_body_shell_mesh()),
        material=black_enamel,
        name="body_shell",
    )
    lens_body.visual(
        _mesh("lens_mount_threads", _build_mount_thread_mesh()),
        material=nickel,
        name="mount_threads",
    )
    lens_body.visual(
        _mesh("lens_helicoid_housing", _build_helicoid_housing_mesh()),
        material=satin_black,
        name="body_helicoid_shell",
    )
    lens_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.029, length=0.050),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    rear_element = model.part("rear_element")
    rear_element.visual(
        Cylinder(radius=0.0182, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0009)),
        material=glass,
        name="rear_element_glass",
    )
    rear_element.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0182, length=0.0018),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.0009)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _mesh("focus_ring_shell", _build_focus_ring_mesh()),
        material=black_enamel,
        name="focus_ring_shell",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.024),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    front_barrel = model.part("front_barrel")
    front_barrel.visual(
        _mesh("front_barrel_shell", _build_front_barrel_mesh()),
        material=satin_black,
        name="front_barrel_shell",
    )
    front_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.046),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    front_element = model.part("front_element")
    front_element.visual(
        _mesh("front_retainer_ring", _build_front_retainer_mesh()),
        material=satin_black,
        name="front_retainer_ring",
    )
    front_element.visual(
        Cylinder(radius=0.0172, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0009)),
        material=glass,
        name="front_element_glass",
    )
    front_element.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.0038),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, -0.0001)),
    )

    model.articulation(
        "body_to_rear_element",
        ArticulationType.FIXED,
        parent=lens_body,
        child=rear_element,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )
    model.articulation(
        "body_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=focus_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(280.0),
        ),
    )
    model.articulation(
        "body_to_front_barrel",
        ArticulationType.PRISMATIC,
        parent=lens_body,
        child=front_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.03,
            lower=0.0,
            upper=0.010,
        ),
    )
    model.articulation(
        "front_barrel_to_front_element",
        ArticulationType.FIXED,
        parent=front_barrel,
        child=front_element,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lens_body = object_model.get_part("lens_body")
    focus_ring = object_model.get_part("focus_ring")
    front_barrel = object_model.get_part("front_barrel")
    front_element = object_model.get_part("front_element")
    focus_joint = object_model.get_articulation("body_to_focus_ring")
    barrel_joint = object_model.get_articulation("body_to_front_barrel")

    barrel_upper = 0.0
    if barrel_joint.motion_limits is not None and barrel_joint.motion_limits.upper is not None:
        barrel_upper = barrel_joint.motion_limits.upper

    ctx.check(
        "focus ring rotates about optical axis",
        tuple(round(value, 6) for value in focus_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={focus_joint.axis}",
    )
    ctx.check(
        "front barrel slides along optical axis",
        tuple(round(value, 6) for value in barrel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={barrel_joint.axis}",
    )
    ctx.expect_origin_distance(
        focus_ring,
        lens_body,
        axes="xy",
        max_dist=1e-6,
        name="focus ring is coaxial with the lens body",
    )
    ctx.expect_within(
        front_barrel,
        lens_body,
        axes="xy",
        inner_elem="front_barrel_shell",
        outer_elem="body_helicoid_shell",
        margin=0.0012,
        name="front barrel stays centered in the helicoid housing",
    )
    ctx.expect_overlap(
        front_barrel,
        lens_body,
        axes="z",
        elem_a="front_barrel_shell",
        elem_b="body_helicoid_shell",
        min_overlap=0.012,
        name="front barrel remains inserted at rest",
    )
    ctx.expect_contact(
        front_element,
        front_barrel,
        elem_a="front_retainer_ring",
        elem_b="front_barrel_shell",
        name="front optical group is seated in the front barrel",
    )

    rest_pos = ctx.part_world_position(front_barrel)
    with ctx.pose({barrel_joint: barrel_upper}):
        ctx.expect_within(
            front_barrel,
            lens_body,
            axes="xy",
            inner_elem="front_barrel_shell",
            outer_elem="body_helicoid_shell",
            margin=0.0012,
            name="extended front barrel stays centered in the helicoid housing",
        )
        ctx.expect_overlap(
            front_barrel,
            lens_body,
            axes="z",
            elem_a="front_barrel_shell",
            elem_b="body_helicoid_shell",
            min_overlap=0.002,
            name="extended front barrel keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(front_barrel)

    ctx.check(
        "front barrel extends forward with focus travel",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.008,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
