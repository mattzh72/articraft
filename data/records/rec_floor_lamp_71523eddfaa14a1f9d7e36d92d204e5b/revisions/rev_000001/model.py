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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_base_mesh():
    return mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.126, 0.000),
                (0.153, 0.004),
                (0.160, 0.010),
                (0.160, 0.016),
                (0.148, 0.023),
                (0.120, 0.029),
                (0.060, 0.033),
                (0.0, 0.033),
            ],
            segments=64,
            closed=True,
        ),
        "torchiere_base_shell",
    )


def _build_shade_mesh():
    return mesh_from_geometry(
        LatheGeometry(
            [
                (0.040, 0.000),
                (0.047, 0.012),
                (0.080, 0.032),
                (0.132, 0.069),
                (0.185, 0.117),
                (0.223, 0.162),
                (0.236, 0.188),
                (0.229, 0.184),
                (0.214, 0.161),
                (0.176, 0.118),
                (0.127, 0.075),
                (0.079, 0.041),
                (0.048, 0.018),
                (0.032, 0.010),
                (0.032, 0.000),
            ],
            segments=72,
            closed=True,
        ),
        "torchiere_shade_shell",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torchiere_floor_lamp")

    cast_iron = model.material("cast_iron", rgba=(0.16, 0.16, 0.17, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.64, 0.66, 0.68, 1.0))
    smoked_metal = model.material("smoked_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.92, 0.91, 0.84, 0.58))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base_mesh = _build_base_mesh()
    shade_mesh = _build_shade_mesh()
    lamp_body = model.part("lamp_body")
    lamp_body.visual(base_mesh, material=cast_iron, name="base_shell")
    lamp_body.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=cast_iron,
        name="base_hub",
    )
    lamp_body.visual(
        Cylinder(radius=0.0105, length=1.660),
        origin=Origin(xyz=(0.0, 0.0, 0.863)),
        material=satin_steel,
        name="post_shaft",
    )
    lamp_body.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.705)),
        material=smoked_metal,
        name="top_socket_stem",
    )
    lamp_body.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 1.695)),
        material=smoked_metal,
        name="shade_mount_ring",
    )
    lamp_body.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 1.708)),
        material=smoked_metal,
        name="shade_fitter",
    )
    lamp_body.visual(
        shade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.690)),
        material=frosted_glass,
        name="shade_shell",
    )
    lamp_body.inertial = Inertial.from_geometry(
        Box((0.48, 0.48, 1.88)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
    )

    dimmer_knob = model.part("dimmer_knob")
    dimmer_knob.visual(
        Box((0.012, 0.022, 0.022)),
        origin=Origin(xyz=(0.0165, 0.0, 0.011)),
        material=knob_black,
        name="knob_collar",
    )
    dimmer_knob.visual(
        Box((0.012, 0.022, 0.022)),
        origin=Origin(xyz=(-0.0165, 0.0, 0.011)),
        material=knob_black,
        name="knob_clamp_left",
    )
    dimmer_knob.visual(
        Box((0.033, 0.007, 0.022)),
        origin=Origin(xyz=(0.0, -0.0140, 0.011)),
        material=knob_black,
        name="knob_clamp_back",
    )
    dimmer_knob.visual(
        Cylinder(radius=0.0125, length=0.016),
        origin=Origin(xyz=(0.0335, 0.0, 0.011)),
        material=knob_black,
        name="knob_wheel",
    )
    dimmer_knob.visual(
        Box((0.008, 0.015, 0.012)),
        origin=Origin(xyz=(0.0460, 0.0, 0.011)),
        material=knob_black,
        name="knob_tab",
    )
    dimmer_knob.inertial = Inertial.from_geometry(
        Box((0.075, 0.040, 0.022)),
        mass=0.08,
        origin=Origin(xyz=(0.012, 0.0, 0.011)),
    )

    model.articulation(
        "post_to_dimmer_knob",
        ArticulationType.CONTINUOUS,
        parent=lamp_body,
        child=dimmer_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.915)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
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
    lamp_body = object_model.get_part("lamp_body")
    dimmer_knob = object_model.get_part("dimmer_knob")
    knob_joint = object_model.get_articulation("post_to_dimmer_knob")

    ctx.check(
        "lamp body exists",
        lamp_body is not None,
        details="lamp_body part lookup failed",
    )
    ctx.check(
        "dimmer knob exists",
        dimmer_knob is not None,
        details="dimmer_knob part lookup failed",
    )
    knob_limits = knob_joint.motion_limits
    ctx.check(
        "dimmer articulation is continuous around post axis",
        knob_joint.joint_type == ArticulationType.CONTINUOUS
        and knob_joint.axis == (0.0, 0.0, 1.0)
        and knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None,
        details=(
            f"type={knob_joint.joint_type}, axis={knob_joint.axis}, "
            f"limits={knob_limits}"
        ),
    )
    ctx.expect_contact(
        dimmer_knob,
        lamp_body,
        elem_a="knob_collar",
        elem_b="post_shaft",
        contact_tol=5e-4,
        name="dimmer collar rides on the post shaft",
    )

    model_aabb = ctx.part_world_aabb(lamp_body)
    base_aabb = ctx.part_element_world_aabb(lamp_body, elem="base_shell")
    shade_aabb = ctx.part_element_world_aabb(lamp_body, elem="shade_shell")
    overall_height = None
    base_width = None
    shade_width = None
    if model_aabb is not None:
        overall_height = model_aabb[1][2] - model_aabb[0][2]
    if base_aabb is not None:
        base_width = base_aabb[1][0] - base_aabb[0][0]
    if shade_aabb is not None:
        shade_width = shade_aabb[1][0] - shade_aabb[0][0]

    ctx.check(
        "lamp has realistic torchiere proportions",
        overall_height is not None
        and 1.75 <= overall_height <= 1.95
        and base_width is not None
        and shade_width is not None
        and shade_width > base_width + 0.10,
        details=(
            f"height={overall_height}, base_width={base_width}, "
            f"shade_width={shade_width}"
        ),
    )

    rest_tab_aabb = ctx.part_element_world_aabb(dimmer_knob, elem="knob_tab")
    rest_tab_center = _aabb_center(rest_tab_aabb)
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_tab_aabb = ctx.part_element_world_aabb(dimmer_knob, elem="knob_tab")
        turned_tab_center = _aabb_center(turned_tab_aabb)
        ctx.check(
            "dimmer knob visibly rotates around the post",
            rest_tab_center is not None
            and turned_tab_center is not None
            and rest_tab_center[0] > 0.020
            and abs(rest_tab_center[1]) < 0.008
            and turned_tab_center[1] > 0.020
            and abs(turned_tab_center[0]) < 0.008,
            details=f"rest={rest_tab_center}, turned={turned_tab_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
