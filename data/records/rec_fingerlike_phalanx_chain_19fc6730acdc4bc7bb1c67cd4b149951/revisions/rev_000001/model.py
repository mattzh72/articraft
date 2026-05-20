from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_JOINT_X = 0.040
BASE_JOINT_Z = 0.006
PROXIMAL_PITCH = 0.060
MIDDLE_PITCH = 0.048
PAD_MOUNT = (0.028, 0.0, -0.0065)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _z_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _profile_solid(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width / 2.0, both=True)


def _cheek(
    x_back: float,
    x_joint: float,
    y_center: float,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
) -> cq.Workplane:
    cheek = _box((x_joint - x_back, width, height), ((x_back + x_joint) / 2.0, y_center, z_center))
    return cheek.union(_y_cylinder(radius, width, (x_joint, y_center, z_center)))


def _make_base_shape() -> cq.Workplane:
    base = _box((0.030, 0.034, 0.030), (0.015, 0.0, 0.0))
    base = base.union(_box((0.014, 0.024, 0.010), (0.023, 0.0, 0.010)))
    base = base.union(_box((0.024, 0.020, 0.008), (0.014, 0.0, -0.012)))
    base = base.cut(_box((0.013, 0.008, 0.012), (0.012, 0.010, 0.004)))
    base = base.cut(_box((0.013, 0.008, 0.012), (0.012, -0.010, 0.004)))
    base = base.cut(_box((0.010, 0.015, 0.004), (0.021, 0.0, 0.011)))

    for y_center in (-0.008, 0.008):
        base = base.union(_cheek(0.030, 0.046, y_center, 0.004, 0.018, 0.004, z_center=BASE_JOINT_Z))

    return base


def _make_proximal_shape() -> cq.Workplane:
    root_tongue = _box((0.010, 0.012, 0.010), (0.004, 0.0, 0.0))
    root_tongue = root_tongue.union(_y_cylinder(0.0038, 0.012, (0.0, 0.0, 0.0)))

    body_profile = [
        (0.008, -0.0045),
        (0.018, -0.0055),
        (0.030, -0.0052),
        (0.040, -0.0022),
        (0.044, 0.0010),
        (0.043, 0.0055),
        (0.035, 0.0082),
        (0.022, 0.0090),
        (0.011, 0.0062),
        (0.008, 0.0040),
    ]
    proximal = root_tongue.union(_profile_solid(body_profile, 0.008))
    proximal = proximal.union(_box((0.016, 0.008, 0.003), (0.028, 0.0, 0.0065)))
    proximal = proximal.cut(_box((0.012, 0.008, 0.0025), (0.026, 0.0, -0.0060)))

    for y_center in (-0.00475, 0.00475):
        proximal = proximal.union(_box((0.006, 0.0035, 0.008), (0.047, y_center, 0.0)))
        proximal = proximal.union(_cheek(0.050, 0.060, y_center, 0.0035, 0.012, 0.0035))

    return proximal


def _make_middle_shape() -> cq.Workplane:
    root_tongue = _box((0.008, 0.006, 0.009), (0.0035, 0.0, 0.0))
    root_tongue = root_tongue.union(_y_cylinder(0.0032, 0.006, (0.0, 0.0, 0.0)))

    body_profile = [
        (0.007, -0.0040),
        (0.015, -0.0048),
        (0.024, -0.0046),
        (0.031, -0.0026),
        (0.036, 0.0008),
        (0.035, 0.0048),
        (0.028, 0.0072),
        (0.018, 0.0078),
        (0.009, 0.0056),
        (0.007, 0.0038),
    ]
    middle = root_tongue.union(_profile_solid(body_profile, 0.006))
    middle = middle.union(_box((0.012, 0.006, 0.0028), (0.019, 0.0, 0.0054)))
    middle = middle.cut(_box((0.009, 0.006, 0.0022), (0.018, 0.0, -0.0053)))

    for y_center in (-0.00375, 0.00375):
        middle = middle.union(_box((0.005, 0.003, 0.007), (0.040, y_center, 0.0)))
        middle = middle.union(_cheek(0.0425, 0.048, y_center, 0.003, 0.010, 0.0030))

    return middle


def _make_distal_shape() -> cq.Workplane:
    root_tongue = _box((0.007, 0.0045, 0.008), (0.0030, 0.0, 0.0))
    root_tongue = root_tongue.union(_y_cylinder(0.0028, 0.0045, (0.0, 0.0, 0.0)))

    body_profile = [
        (0.006, -0.0035),
        (0.014, -0.0042),
        (0.022, -0.0040),
        (0.030, -0.0032),
        (0.034, -0.0020),
        (0.034, 0.0036),
        (0.029, 0.0052),
        (0.019, 0.0057),
        (0.009, 0.0046),
        (0.006, 0.0030),
    ]
    distal = root_tongue.union(_profile_solid(body_profile, 0.005))
    distal = distal.union(_box((0.012, 0.010, 0.009), (0.030, 0.0, 0.000)))
    distal = distal.union(_box((0.010, 0.010, 0.003), (0.029, 0.0, 0.0055)))
    distal = distal.cut(_box((0.007, 0.005, 0.0022), (0.015, 0.0, -0.0050)))
    return distal


def _make_tip_pad_shape() -> cq.Workplane:
    pad = _box((0.016, 0.010, 0.004), (0.0, 0.0, 0.0))
    for x_center in (-0.005, 0.005):
        pad = pad.cut(_z_cylinder(0.0016, 0.004, (x_center, 0.0, 0.0)))
    return pad


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_grasping_finger")

    model.material("base_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("link_metal", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("pad_rubber", rgba=(0.12, 0.13, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "finger_base"),
        material="base_dark",
        name="base_body",
    )

    proximal = model.part("proximal_link")
    proximal.visual(
        mesh_from_cadquery(_make_proximal_shape(), "finger_proximal"),
        material="link_metal",
        name="proximal_body",
    )

    middle = model.part("middle_link")
    middle.visual(
        mesh_from_cadquery(_make_middle_shape(), "finger_middle"),
        material="link_metal",
        name="middle_body",
    )

    distal = model.part("distal_link")
    distal.visual(
        mesh_from_cadquery(_make_distal_shape(), "finger_distal"),
        material="link_metal",
        name="distal_body",
    )

    tip_pad = model.part("tip_pad")
    tip_pad.visual(
        mesh_from_cadquery(_make_tip_pad_shape(), "finger_tip_pad"),
        material="pad_rubber",
        name="pad_body",
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(BASE_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=1.00,
        ),
    )
    model.articulation(
        "distal_to_tip_pad",
        ArticulationType.FIXED,
        parent=distal,
        child=tip_pad,
        origin=Origin(xyz=PAD_MOUNT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    proximal = object_model.get_part("proximal_link")
    middle = object_model.get_part("middle_link")
    distal = object_model.get_part("distal_link")
    tip_pad = object_model.get_part("tip_pad")

    base_to_proximal = object_model.get_articulation("base_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        base,
        proximal,
        reason="Root hinge barrels and the unmodeled through-pin share a hidden journal envelope inside the clevis.",
    )
    ctx.allow_overlap(
        proximal,
        middle,
        reason="Middle hinge uses an interleaved clevis-and-tongue joint represented without a separate pin body.",
    )
    ctx.allow_overlap(
        middle,
        distal,
        reason="Distal hinge uses the same compact journal abstraction, so adjacent link solids intentionally share the concealed pin envelope.",
    )

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
        "three serial finger joints bend in one plane",
        all(joint.axis == (0.0, 1.0, 0.0) for joint in (base_to_proximal, proximal_to_middle, middle_to_distal)),
        details="Expected all three serial revolute axes to run along +Y for planar curling.",
    )

    ctx.expect_contact(
        base,
        proximal,
        contact_tol=5e-4,
        name="base clevis supports proximal link",
    )
    ctx.expect_contact(
        proximal,
        middle,
        contact_tol=5e-4,
        name="proximal fork supports middle link",
    )
    ctx.expect_contact(
        middle,
        distal,
        contact_tol=5e-4,
        name="middle fork supports distal link",
    )
    ctx.expect_contact(
        distal,
        tip_pad,
        contact_tol=5e-4,
        name="tip pad is mounted to distal carrier",
    )
    ctx.expect_overlap(
        distal,
        tip_pad,
        axes="xy",
        min_overlap=0.010,
        name="tip pad footprint stays seated on distal face",
    )

    def _size_x(part_name: str) -> float:
        aabb = ctx.part_world_aabb(part_name)
        assert aabb is not None
        return aabb[1][0] - aabb[0][0]

    def _size_y(part_name: str) -> float:
        aabb = ctx.part_world_aabb(part_name)
        assert aabb is not None
        return aabb[1][1] - aabb[0][1]

    ctx.check(
        "moving links step down away from root",
        _size_x("proximal_link") > _size_x("middle_link") > _size_x("distal_link")
        and _size_y("proximal_link") > _size_y("middle_link") > _size_y("distal_link"),
        details=(
            f"x sizes: proximal={_size_x('proximal_link'):.4f}, "
            f"middle={_size_x('middle_link'):.4f}, distal={_size_x('distal_link'):.4f}; "
            f"y sizes: proximal={_size_y('proximal_link'):.4f}, "
            f"middle={_size_y('middle_link'):.4f}, distal={_size_y('distal_link'):.4f}"
        ),
    )

    with ctx.pose(
        {
            base_to_proximal: 0.90,
            proximal_to_middle: 0.72,
            middle_to_distal: 0.52,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no clipping in grasp pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
